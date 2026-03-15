package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class DriverControlsRed {

    private final Follower follower;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    private final Pose parkingRed = new Pose(-25, -30, 0);
    private PathChain parkingRedPath;

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

    private long nextSteadyRefreshMs = 0;
    private long nextPulseAllowedMs = 0;

    private static final double ALIGN_TOLERANCE_DEG = 1.3;
    private static final double STEADY_RUMBLE_POWER = 0.35;
    private static final int STEADY_RUMBLE_MS = 10000;
    private static final int STEADY_REFRESH_EARLY_MS = 9000;
    private static final int PULSE_INTERVAL_MS = 250;

    public DriverControlsRed(Follower follower, Telemetry telemetry, Limelight limelight) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;

        if (this.limelight != null) this.limelight.setTargetRed();

        steadyRumbleEffect = new RumbleEffect.Builder()
                .addStep(0.0, STEADY_RUMBLE_POWER, STEADY_RUMBLE_MS)
                .build();

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

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        if (btnOptions.wasPressed(gamepad1.options)) {
            resetRobotPose();
            updateAutoAlignRumble(gamepad1);
            return;
        }

        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;
            if (homingMechanismEngaged) {
                buildParkingPathOnce();
                follower.followPath(parkingRedPath);
                homingPathStarted = true;
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            updateAutoAlignRumble(gamepad1);
            return;
        }

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

        if (limelight != null) {
            if (autoAlignEnabled) {
                double fieldY = pose.getY();
                double desiredTx = getRedDesiredTxFromFieldY(fieldY);
                limelight.setTargetAngle(desiredTx);
            }

            limelight.update();
        }

        if (autoAlignEnabled) {
            boolean targetVisible = (limelight != null && limelight.isTargetVisible());

            if (targetVisible) {
                lastVisionTurn = (limelight != null) ? limelight.getTurnPower() : 0.0;
                lastVisionTurn = clamp(lastVisionTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                turn = lastVisionTurn;
                usingAutoTurn = true;
                lastTurnSource = "VISION_X";
            } else {
                double targetHeading = calculateTargetHeading(pose.getY());
                double currentHeading = pose.getHeading();

                double headingError = wrapAngleRad(targetHeading - currentHeading);
                double blindTurn = HEADING_kP * headingError;
                blindTurn = clamp(blindTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                lastVisionTurn = blindTurn;
                turn = blindTurn;
                usingAutoTurn = true;
                lastTurnSource = "P";
            }
        } else {
            lastTurnSource = "MANUAL";
        }

        updateAutoAlignRumble(gamepad1);
        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void updateAutoAlignRumble(Gamepad gamepad1) {
        long now = System.currentTimeMillis();

        if (!autoAlignEnabled) {
            if (rumbleMode != RumbleMode.OFF) {
                rumbleMode = RumbleMode.OFF;
                gamepad1.stopRumble();
            }
            return;
        }

        if (rumbleMode != RumbleMode.FAST_PULSE || now >= nextPulseAllowedMs) {
            rumbleMode = RumbleMode.FAST_PULSE;
            gamepad1.runRumbleEffect(fastPulseEffect);
            nextPulseAllowedMs = now + PULSE_INTERVAL_MS;
        }
    }

    private void resetRobotPose() {
        homingMechanismEngaged = false;
        homingPathStarted = false;

        follower.startTeleopDrive();

        // Adjust this if your red-side reset pose should be different
        follower.setPose(new Pose(-45, 120, Math.toRadians(-40)));

        if (limelight != null) {
            limelight.setTargetAngle(0);
        }

        lastTurnSource = "POSE_RESET";
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingAutoTurn) {
        double translationScale = slowMode ? NORMAL_SPEED : 1.0;
        double rotationScale = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.5);

        lastTranslationScale = translationScale;
        lastRotationScale = rotationScale;

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, false);
    }

    private double getRedDesiredTxFromFieldY(double fieldY) {
        // Mirrored from blue:
        // Blue: Y < -96 => -3.0, Y > -48 => 0.4, else 0.0
        // Red : Y >  96 => +3.0, Y <  48 => -0.4, else 0.0
        if (fieldY > 96) {
            return 3.0;
        } else if (fieldY < 48.0) {
            return -0.4;
        } else {
            return 0.0;
        }
    }

    public double calculateTargetHeading(double fieldY) {
        // Mirrored from blue:
        // Blue: < -96 => 160, > -48 => 110, else 135
        // Red : >  96 => 20,  <  48 => 70,  else 45
        if (fieldY > 96) {
            return Math.toRadians(-155);
        } else if (fieldY < 48.0) {
            return Math.toRadians(-110);
        } else {
            return Math.toRadians(-135);
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
            telemetry.addData("Locked", limelight.isCenteredOnTarget(ALIGN_TOLERANCE_DEG));
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

        parkingRedPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingRed))
                .setLinearHeadingInterpolation(pose.getHeading(), 0.0)
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();
    }
}