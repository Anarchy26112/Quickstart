package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class DriverControlsBlue {

    private final Follower follower;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    private final Pose parkingBlue = new Pose(25, -30, 0);
    private PathChain parkingBluePath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    // --- Cached values for Telemetry ---
    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    // =========================
    // RUMBLE (AUTO-ALIGN FEEDBACK)
    // =========================
    private enum RumbleMode { OFF, STEADY, FAST_PULSE }
    private RumbleMode rumbleMode = RumbleMode.OFF;

    private RumbleEffect steadyRumbleEffect;
    private RumbleEffect fastPulseEffect;

    // refresh steady rumble periodically (effects have finite duration)
    private long nextSteadyRefreshMs = 0;
    private long nextPulseAllowedMs = 0;

    // tune these
    private static final double ALIGN_TOLERANCE_DEG = 1.3;  // "perfectly aligned" tolerance
    private static final double STEADY_RUMBLE_POWER = 0.35; // gentle continuous rumble
    private static final int STEADY_RUMBLE_MS = 10000;      // 10s steady, then we refresh
    private static final int STEADY_REFRESH_EARLY_MS = 9000;// refresh a bit early
    private static final int PULSE_INTERVAL_MS = 250;       // how often we allow a pulse sequence

    public DriverControlsBlue(Follower follower, Telemetry telemetry, Limelight limelight) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;

        if (this.limelight != null) this.limelight.setTargetBlue();

        // Build rumble effects
        steadyRumbleEffect = new RumbleEffect.Builder()
                .addStep(0.0, STEADY_RUMBLE_POWER, STEADY_RUMBLE_MS) // right motor steady
                .build();

        // Fast "locked" pulses (bzz-bzz-bzz)
        fastPulseEffect = new RumbleEffect.Builder()
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .build();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        Pose pose = follower.getPose();
        // 2) Handle Inputs (Slow Mode / Auto Align Toggles)
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        // Pose Reset (OPTIONS) - HIGH PRIORITY SAFETY RESET
        if (btnOptions.wasPressed(gamepad1.options)) {
            resetRobotPose();
            updateAutoAlignRumble(gamepad1); // keep rumble state correct even on early return
            return;
        }

        // 3) Homing / Parking Logic (Highest Priority)
        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;
            if (homingMechanismEngaged) {
                buildParkingPathOnce();
                follower.followPath(parkingBluePath);
                homingPathStarted = true;
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            updateAutoAlignRumble(gamepad1); // still keep rumble synced
            return; // Exit early, let path follower run
        }

        // 4) Read Driver Sticks
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false; // if true, rotationScale = 1.0

        if (limelight != null) {
            // If we're in auto-align, compute desiredTx now so update() uses it immediately
            if (autoAlignEnabled) {
                double fieldY = pose.getY();
                double desiredTx = getBlueDesiredTxFromFieldX(fieldY);
                limelight.setTargetAngle(desiredTx);
            }

            // Now read camera + compute PID using the correct setpoint for THIS loop
            limelight.update();
        }

        // 5) Auto-Align Logic (after limelight.update(), so data is fresh)
        if (autoAlignEnabled) {
            boolean targetVisible = (limelight != null && limelight.isTargetVisible());

            if (targetVisible) {
                // CASE A: Target Visible -> Use Vision Turn (PID already computed this loop)
                lastVisionTurn = (limelight != null) ? limelight.getTurnPower() : 0.0;

                // Clamp vision turn so it doesn't whip
                lastVisionTurn = clamp(lastVisionTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                turn = lastVisionTurn;
                usingAutoTurn = true;
                lastTurnSource = "VISION_X";

            } else {
                // CASE B: Target NOT Visible -> P Turn using odometry aim
                double targetHeading = calculateTargetHeading(pose.getY());
                double currentHeading = pose.getHeading();

                double headingError = wrapAngleRad(targetHeading - currentHeading);

                double blindTurn = HEADING_kP * headingError;

                // Clamp blind P-turn so it doesn't whip
                blindTurn = clamp(blindTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                lastVisionTurn = blindTurn;
                turn = blindTurn;
                usingAutoTurn = true;
                lastTurnSource = "P";
            }
        } else {
            lastTurnSource = "MANUAL";
        }

        // Update rumble AFTER limelight.update() so "centered" is based on fresh error
        updateAutoAlignRumble(gamepad1);

        // 6) Apply Final Drive Powers
        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void updateAutoAlignRumble(Gamepad gamepad1) {
        long now = System.currentTimeMillis();

        // If auto-align is OFF, ensure rumble is OFF
        if (!autoAlignEnabled) {
            if (rumbleMode != RumbleMode.OFF) {
                rumbleMode = RumbleMode.OFF;
                gamepad1.stopRumble();
            }
            return;
        }

        // Auto-align is ON -> always run the fast pulse effect (single mode, no deadband logic)
        // Gate it so it doesn't restart every loop (prevents "stuttery" feeling)
        if (rumbleMode != RumbleMode.FAST_PULSE || now >= nextPulseAllowedMs) {
            rumbleMode = RumbleMode.FAST_PULSE;
            gamepad1.runRumbleEffect(fastPulseEffect);
            nextPulseAllowedMs = now + PULSE_INTERVAL_MS;
        }
    }


    private void resetRobotPose() {
        // Stop any active path/homing state so nothing fights the pose reset
        homingMechanismEngaged = false;
        homingPathStarted = false;

        // Ensure follower is back in teleop drive mode
        follower.startTeleopDrive();

        // Reset pose to requested location/orientation
        follower.setPose(new Pose(45, -120, Math.toRadians(140)));

        // Optional: clear limelight setpoint so PID doesn't jump
        if (limelight != null) {
            limelight.setTargetAngle(0);
        }

        lastTurnSource = "POSE_RESET";
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingAutoTurn) {
        double translationScale = slowMode ? NORMAL_SPEED : 1.0;

        // If auto-turn (vision or blind P), give full rotation power; otherwise scale for driver
        double rotationScale = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.5);

        lastTranslationScale = translationScale;
        lastRotationScale = rotationScale;

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, false);
    }

    private double getBlueDesiredTxFromFieldX(double fieldY) {
        if (fieldY < -96) {
            return -3.0;
        } else if (fieldY > -48.0) {
            return 1.0;
        } else {
            return 0.0;
        }
    }

    // Odometry-based aim point -> heading (radians)
    public double calculateTargetHeading(double fieldY) {
        if (fieldY < -96) {
            return Math.toRadians(160);
        } else if (fieldY > -48.0) {
            return Math.toRadians(110);
        } else {
            return Math.toRadians(135);
        }
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void updateTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("Drive Mode", slowMode
                ? String.format(Locale.US, "Slow (%.0f%%)", NORMAL_SPEED * 100.0)
                : "Full (100%)");

        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");
        telemetry.addData("State", homingMechanismEngaged ? "HOMING" : "TELEOP");

        telemetry.addData("Turn Source", lastTurnSource);
        telemetry.addData("Auto Turn (Raw)", "%.3f", lastVisionTurn);
        telemetry.addData("Turn (Applied)", "%.3f", lastAppliedTurn);

        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag " + limelight.getDetectedTagId());
            telemetry.addData("tx", "%.2f", limelight.getTx());
        }

        telemetry.addData("Pose", String.format(Locale.US, "X:%.1f Y:%.1f H:%.1f°",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())));
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    private void buildParkingPathOnce() {
        Pose pose = follower.getPose();
        Pose currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());

        parkingBluePath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingBlue))
                .setLinearHeadingInterpolation(pose.getHeading(), 0.0)
                .setVelocityConstraint(0.025) // Very slow approach
                .setBrakingStrength(2) // Strong braking
                .build();
    }
}

