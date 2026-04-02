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

public class DriverControlsBlue {

    private final Follower follower;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;

    // Intake-state flag passed in from OperatorControls / OpMode
    private boolean intakingActive = false;

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
    // INTAKE AIM
    // =========================
    private static final double INTAKE_AIM_TARGET_DEG = -26.0;
    private static final double INTAKE_AIM_TARGET_RAD = Math.toRadians(INTAKE_AIM_TARGET_DEG);
    private static final double INTAKE_AIM_MAX_TURN = 0.35;
    private static final double INTAKE_AIM_DEADBAND_RAD = Math.toRadians(1.0);
    private static final double ODOM_AIM_DEADBAND_RAD =
            Math.toRadians(ODOM_AIM_DEADBAND_DEG);

    // =========================
    // RUMBLE (AUTO-ALIGN FEEDBACK)
    // =========================
    private enum RumbleMode { OFF, FAST_PULSE }
    private RumbleMode rumbleMode = RumbleMode.OFF;

    private RumbleEffect fastPulseEffect;
    private long nextPulseAllowedMs = 0;

    private static final int PULSE_INTERVAL_MS = 250;

    public DriverControlsBlue(Follower follower, Telemetry telemetry, Limelight limelight) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;

        if (this.limelight != null) this.limelight.setTargetBlue();

        fastPulseEffect = new RumbleEffect.Builder()
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .build();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void setIntakingActive(boolean intakingActive) {
        this.intakingActive = intakingActive;
    }

    public void update(Gamepad gamepad1) {
        Pose pose = follower.getPose();

        // Driver toggle for auto-align
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        // Driver toggle for slow mode
        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        // Pose Reset (OPTIONS) - HIGH PRIORITY SAFETY RESET
        if (btnOptions.wasPressed(gamepad1.options)) {
            resetRobotPose();
            updateAutoAlignRumble(gamepad1);
            return;
        }

        // Homing / Parking Logic
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
            updateAutoAlignRumble(gamepad1);
            return;
        }

        // Read Driver Sticks
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

        // Square button heading assist: only while intaking
        boolean squareAimActive = gamepad1.square && intakingActive;

        if (squareAimActive) {
            double currentHeading = pose.getHeading();
            double headingError = wrapAngleRad(INTAKE_AIM_TARGET_RAD - currentHeading);

            double turnCommand = HEADING_kP * headingError;

            // static feedforward only when error is meaningful
            if (Math.abs(headingError) > INTAKE_AIM_DEADBAND_RAD) {
                turnCommand += Math.signum(headingError) * kS_VOLTAGE_COMP;
            }

            turn = clamp(turnCommand, -INTAKE_AIM_MAX_TURN, INTAKE_AIM_MAX_TURN);
            usingAutoTurn = true;
            lastVisionTurn = turn;
            lastTurnSource = "INTAKE_30deg";
        } else {
            if (limelight != null) {
                if (autoAlignEnabled) {
                    double fieldY = pose.getY();
                    double desiredTx = getBlueDesiredTxFromFieldX(fieldY);
                    limelight.setTargetAngle(desiredTx);
                }

                limelight.update();
            }

            // Auto-Align Logic
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

                    if (Math.abs(headingError) > ODOM_AIM_DEADBAND_RAD) {
                        blindTurn += Math.signum(headingError) * kS_VOLTAGE_COMP;
                    }

                    blindTurn = clamp(blindTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    lastVisionTurn = blindTurn;
                    turn = blindTurn;
                    usingAutoTurn = true;
                    lastTurnSource = "ODOM_P+FF";
                }
            }
        }

        updateAutoAlignRumble(gamepad1);

        // Apply Final Drive Powers
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
        follower.setPose(new Pose(45, -120, Math.toRadians(140)));

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

    private double getBlueDesiredTxFromFieldX(double fieldY) {
        if (fieldY < -96) {
            return -4.6;
        } else if (fieldY < -84) {
            return -2.0;
        } else if (fieldY > -48.0) {
            return 0.4;
        } else {
            return 0;
        }
    }

    public double calculateTargetHeading(double fieldY) {
        if (fieldY < -96) {
            return Math.toRadians(180);
        } else if (fieldY < -84) {
            return Math.toRadians(150);
        } else if (fieldY > -48.0) {
            return Math.toRadians(110);
        } else {
            return Math.toRadians(130);
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
        telemetry.addData("Intaking Active", intakingActive);

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

    public void forceEnableAutoAlign() {
        autoAlignEnabled = true;
        rumbleMode = RumbleMode.OFF;
        nextPulseAllowedMs = 0;
    }

    public void forceDisableAutoAlign() {
        autoAlignEnabled = false;
        rumbleMode = RumbleMode.OFF;
    }

    private void buildParkingPathOnce() {
        Pose pose = follower.getPose();
        Pose currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());

        parkingBluePath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingBlue))
                .setLinearHeadingInterpolation(pose.getHeading(), 0.0)
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();
    }
}