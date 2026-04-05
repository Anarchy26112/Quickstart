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
    private boolean intakingActive = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    private final Pose parkingBlue = new Pose(25, -30, 0);
    private PathChain parkingBluePath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    // Cached values for telemetry
    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    // Intake aim
    private static final double INTAKE_AIM_TARGET_DEG = -26.0;
    private static final double INTAKE_AIM_TARGET_RAD = Math.toRadians(INTAKE_AIM_TARGET_DEG);
    private static final double INTAKE_AIM_MAX_TURN = 0.35;
    private static final double INTAKE_AIM_DEADBAND_RAD = Math.toRadians(1.0);
    private static final double ODOM_AIM_DEADBAND_RAD =
            Math.toRadians(ODOM_AIM_DEADBAND_DEG);

    // Limelight polling cadence
    private static final long LIMELIGHT_IDLE_POLL_MS = 100;   // 10 Hz when not aligning
    private static final long LIMELIGHT_ALIGN_POLL_MS = 0;    // as fast as loop when aligning
    private long lastLimelightIdlePollMs = 0;
    private long lastLimelightAlignPollMs = 0;

    // Rumble
    private enum RumbleMode { OFF, FAST_PULSE }
    private RumbleMode rumbleMode = RumbleMode.OFF;

    private final RumbleEffect fastPulseEffect;
    private long nextPulseAllowedMs = 0;

    private static final int PULSE_INTERVAL_MS = 250;

    public DriverControlsBlue(Follower follower, Telemetry telemetry, Limelight limelight) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;

        if (this.limelight != null) {
            this.limelight.setTargetBlue();
        }

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

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    public void forceEnableAutoAlign() {
        autoAlignEnabled = true;
        lastLimelightAlignPollMs = 0;
        lastLimelightIdlePollMs = 0;

        if (limelight != null) {
            limelight.setTargetAngle(0.0);
        }
    }

    public void forceDisableAutoAlign() {
        autoAlignEnabled = false;
        lastLimelightAlignPollMs = 0;
    }

    public void update(Gamepad gamepad1, Pose pose, long nowMs) {
        if (pose == null) return;

        // Push field position into Limelight so it can switch gain profiles
        if (limelight != null) {
            limelight.setRobotY(pose.getY());
        }

        // -------------------------------
        // Button toggles / one-shot actions
        // -------------------------------
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
            lastLimelightAlignPollMs = 0;
            lastLimelightIdlePollMs = 0;

            if (limelight != null) {
                limelight.setTargetAngle(0.0);
            }
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        if (btnOptions.wasPressed(gamepad1.options)) {
            resetRobotPose();
            updateAutoAlignRumble(gamepad1, nowMs);
            return;
        }

        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;

            if (homingMechanismEngaged) {
                buildParkingPath();
                follower.followPath(parkingBluePath);
                homingPathStarted = true;
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            updateAutoAlignRumble(gamepad1, nowMs);
            return;
        }

        // -------------------------------
        // Read sticks
        // -------------------------------
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

        // -------------------------------
        // Intake heading assist (square)
        // -------------------------------
        boolean squareAimActive = gamepad1.square && intakingActive;

        if (squareAimActive) {
            double currentHeading = pose.getHeading();
            double headingError = wrapAngleRad(INTAKE_AIM_TARGET_RAD - currentHeading);

            double turnCommand = HEADING_kP * headingError;

            if (Math.abs(headingError) > INTAKE_AIM_DEADBAND_RAD) {
                turnCommand += Math.signum(headingError) * FAST_kS_VOLTAGE_COMP;
            }

            turn = clamp(turnCommand, -INTAKE_AIM_MAX_TURN, INTAKE_AIM_MAX_TURN);
            usingAutoTurn = true;
            lastVisionTurn = turn;
            lastTurnSource = "INTAKE_30deg";

            updateAutoAlignRumble(gamepad1, nowMs);
            applyScaledDrive(drive, strafe, turn, usingAutoTurn);
            return;
        }

        if (limelight != null) {
            if (autoAlignEnabled) {
                double desiredTx = getBlueDesiredTxFromFieldX(pose.getY());
                limelight.setTargetAngle(desiredTx);

                if (nowMs - lastLimelightAlignPollMs >= LIMELIGHT_ALIGN_POLL_MS) {
                    limelight.pollVision();
                    lastLimelightAlignPollMs = nowMs;
                }
            } else if (nowMs - lastLimelightIdlePollMs >= LIMELIGHT_IDLE_POLL_MS) {
                limelight.pollVision();
                lastLimelightIdlePollMs = nowMs;
            }

            limelight.updateControl();
        }

        // -------------------------------
        // Auto-align turn selection
        // -------------------------------
        if (autoAlignEnabled) {
            boolean targetVisible = limelight != null && limelight.isTargetVisible();

            if (targetVisible) {
                double visionTurn = limelight.getTurnPower();
                visionTurn = clamp(visionTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                lastVisionTurn = visionTurn;
                turn = visionTurn;
                usingAutoTurn = true;
                lastTurnSource = "VISION_X";
            } else {
                double targetHeading = calculateTargetHeading(pose.getY());
                double currentHeading = pose.getHeading();

                double headingError = wrapAngleRad(targetHeading - currentHeading);
                double blindTurn = HEADING_kP * headingError;

                if (Math.abs(headingError) > ODOM_AIM_DEADBAND_RAD) {
                    blindTurn += Math.signum(headingError) * FAST_kS_VOLTAGE_COMP;
                }

                blindTurn = clamp(blindTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                lastVisionTurn = blindTurn;
                turn = blindTurn;
                usingAutoTurn = true;
                lastTurnSource = "ODOM_P+FF";
            }
        } else {
            lastTurnSource = "MANUAL";
        }

        // -------------------------------
        // Output
        // -------------------------------
        updateAutoAlignRumble(gamepad1, nowMs);
        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void updateAutoAlignRumble(Gamepad gamepad1, long nowMs) {
        if (!autoAlignEnabled) {
            if (rumbleMode != RumbleMode.OFF) {
                rumbleMode = RumbleMode.OFF;
                gamepad1.stopRumble();
            }
            return;
        }

        if (rumbleMode != RumbleMode.FAST_PULSE || nowMs >= nextPulseAllowedMs) {
            rumbleMode = RumbleMode.FAST_PULSE;
            gamepad1.runRumbleEffect(fastPulseEffect);
            nextPulseAllowedMs = nowMs + PULSE_INTERVAL_MS;
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
        lastLimelightAlignPollMs = 0;
        lastLimelightIdlePollMs = 0;
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
            return 0.35;
        } else {
            return 0.0;
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

    private void buildParkingPath() {
        Pose startPose = follower.getPose();
        if (startPose == null) {
            startPose = new Pose(45, -120, Math.toRadians(140));
        }

        parkingBluePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkingBlue))
                .build();
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void updateTelemetry(Pose pose) {
        if (pose == null) return;

        telemetry.addData("Drive Mode", slowMode
                ? String.format(Locale.US, "Slow (%.0f%%)", NORMAL_SPEED * 100.0)
                : "Full (100%)");

        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");
        telemetry.addData("State", homingMechanismEngaged ? "HOMING" : "TELEOP");
        telemetry.addData("Intaking Active", intakingActive);

        telemetry.addData("Turn Source", lastTurnSource);
        telemetry.addData("Auto Turn (Raw)", "%.3f", lastVisionTurn);
        telemetry.addData("Turn (Applied)", "%.3f", lastAppliedTurn);
        telemetry.addData("Drive Input", "%.2f", lastDrive);
        telemetry.addData("Strafe Input", "%.2f", lastStrafe);
        telemetry.addData("Translation Scale", "%.2f", lastTranslationScale);
        telemetry.addData("Rotation Scale", "%.2f", lastRotationScale);

        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag %d", limelight.getDetectedTagId());
            telemetry.addData("tx", "%.2f", limelight.getTx());
            telemetry.addData("LL Turn", "%.3f", limelight.getTurnPower());
            telemetry.addData("LL Rate", "%.2f", limelight.getFilteredRate());
            telemetry.addData("LL Error", "%.2f", limelight.getLastError());
            telemetry.addData("LL Fresh", limelight.isFreshFrameThisLoop());
            telemetry.addData("LL Settled", limelight.isSettled());
            telemetry.addData("LL ShootReady", limelight.isShootReady());
            telemetry.addData("LL AimProfile", limelight.getAimProfileName());
        } else if (autoAlignEnabled) {
            telemetry.addData("Aligning To", "ODOM fallback");
        }

        telemetry.addData("Pose X", "%.1f", pose.getX());
        telemetry.addData("Pose Y", "%.1f", pose.getY());

        if (autoAlignEnabled) {
            double targetHeadingRad = calculateTargetHeading(pose.getY());
            double targetHeadingDeg = Math.toDegrees(targetHeadingRad);
            double headingErrorDeg = Math.toDegrees(
                    wrapAngleRad(targetHeadingRad - pose.getHeading())
            );

            telemetry.addData("AutoAim Target Heading Deg", "%.1f", targetHeadingDeg);
            telemetry.addData("AutoAim Heading Error Deg", "%.1f", headingErrorDeg);
        }
    }
}