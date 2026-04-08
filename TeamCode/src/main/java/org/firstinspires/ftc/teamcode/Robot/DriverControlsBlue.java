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
    private final GoalAimController aimController;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;
    private boolean intakingActive = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();
    private final ButtonHelper btnTriangle = new ButtonHelper();

    private final Pose parkingBlue = new Pose(25, -30, 0);
    private PathChain parkingBluePath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    // One-shot Limelight relocalization request
    private boolean relocalizeRequested = false;

    // V2 triangle state
    private boolean triangleWritebackSeen = false;
    private long triangleSettledSinceMs = 0;
    private boolean triangleTimedOut = false;

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

    // Limelight polling cadence
    private static final long LIMELIGHT_IDLE_POLL_MS = 100;
    private static final long LIMELIGHT_ALIGN_POLL_MS = 10;
    private long lastLimelightIdlePollMs = 0;
    private long lastLimelightAlignPollMs = 0;

    // Slightly longer timeout for triangle snap
    private static final long RELOCALIZE_TIMEOUT_MS = 1800;
    private long relocalizeStartMs = 0;

    // End triangle mode once heading is truly settled
    private static final double TRIANGLE_FINISH_HEADING_ERROR_DEG = 0.8;
    private static final double TRIANGLE_FINISH_TURN_POWER = 0.04;
    private static final long TRIANGLE_SETTLE_HOLD_MS = 120;

    // Translation slow-down during triangle snap
    private static final double TRIANGLE_TRANSLATION_SCALE = 0.35;
    private static final double TRIANGLE_TRANSLATION_SCALE_NO_TAG = 0.20;

    // Rumble
    private enum RumbleMode { OFF, FAST_PULSE }
    private RumbleMode rumbleMode = RumbleMode.OFF;

    private final RumbleEffect fastPulseEffect;
    private long nextPulseAllowedMs = 0;

    private static final int PULSE_INTERVAL_MS = 250;

    public DriverControlsBlue(Follower follower, Telemetry telemetry, GoalAimController aimController) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimController = aimController;

        if (this.aimController != null) {
            this.aimController.setTargetBlue();
            this.aimController.setGoal(GOAL_X, GOAL_Y);
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
    }

    public void forceDisableAutoAlign() {
        autoAlignEnabled = false;
        clearTriangleState();

        if (aimController != null) {
            aimController.setUseVisionCorrection(false);
            aimController.setForceVisionRelocalization(false);
        }
    }

    public void update(Gamepad gamepad1, Pose pose, long nowMs) {
        if (pose == null) return;

        if (aimController != null) {
            aimController.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());
            aimController.setGoal(GOAL_X, GOAL_Y);
        }

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;

            if (!autoAlignEnabled && aimController != null) {
                aimController.setUseVisionCorrection(relocalizeRequested);
                aimController.setForceVisionRelocalization(relocalizeRequested);
            }

            lastLimelightAlignPollMs = 0;
            lastLimelightIdlePollMs = 0;
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

        if (btnTriangle.wasPressed(gamepad1.triangle)) {
            relocalizeRequested = true;
            relocalizeStartMs = nowMs;
            triangleWritebackSeen = false;
            triangleSettledSinceMs = 0;
            triangleTimedOut = false;

            lastLimelightAlignPollMs = 0;
            lastLimelightIdlePollMs = 0;

            if (aimController != null) {
                aimController.setUseVisionCorrection(true);
                aimController.setForceVisionRelocalization(true);
                aimController.noteTriangleStart();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            updateAutoAlignRumble(gamepad1, nowMs);
            return;
        }

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

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

        boolean forceRelocalizeActive = relocalizeRequested;

        if (aimController != null) {
            if (forceRelocalizeActive) {
                if (nowMs - lastLimelightAlignPollMs >= LIMELIGHT_ALIGN_POLL_MS) {
                    aimController.pollVision();
                    lastLimelightAlignPollMs = nowMs;
                }
            } else if (nowMs - lastLimelightIdlePollMs >= LIMELIGHT_IDLE_POLL_MS) {
                aimController.pollVision();
                lastLimelightIdlePollMs = nowMs;
            }

            aimController.setUseVisionCorrection(forceRelocalizeActive);
            aimController.setForceVisionRelocalization(forceRelocalizeActive);
            aimController.update();

            boolean snapFinished =
                    Math.abs(aimController.getHeadingErrorDeg()) <= TRIANGLE_FINISH_HEADING_ERROR_DEG
                            && Math.abs(aimController.getTurnPower()) <= TRIANGLE_FINISH_TURN_POWER;

            if (aimController.didApplyVisionWriteback()) {
                triangleWritebackSeen = true;
            }

            if (relocalizeRequested) {
                if (triangleWritebackSeen && snapFinished) {
                    if (triangleSettledSinceMs == 0) {
                        triangleSettledSinceMs = nowMs;
                    } else if ((nowMs - triangleSettledSinceMs) >= TRIANGLE_SETTLE_HOLD_MS) {
                        clearTriangleState();
                        aimController.setUseVisionCorrection(false);
                        aimController.setForceVisionRelocalization(false);
                    }
                } else {
                    triangleSettledSinceMs = 0;
                }
            }

            if (relocalizeRequested && relocalizeStartMs > 0
                    && (nowMs - relocalizeStartMs) >= RELOCALIZE_TIMEOUT_MS) {
                triangleTimedOut = true;
                clearTriangleState();
                aimController.setUseVisionCorrection(false);
                aimController.setForceVisionRelocalization(false);
            }
        }

        boolean snapHeadingAssistActive = autoAlignEnabled || forceRelocalizeActive;

        if (forceRelocalizeActive) {
            turn = 0.0;
        }

        if (snapHeadingAssistActive && aimController != null) {
            double autoTurn = aimController.getTurnPower();
            autoTurn = clamp(autoTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            lastVisionTurn = autoTurn;
            turn = autoTurn;
            usingAutoTurn = true;

            if (forceRelocalizeActive && aimController.isTargetVisible()) {
                lastTurnSource = triangleWritebackSeen ? "TRIANGLE_SETTLING" : "TRIANGLE_SNAP+LL";
            } else if (forceRelocalizeActive) {
                lastTurnSource = "TRIANGLE_WAITING_FOR_TAG";
            } else {
                lastTurnSource = "ODOM_PD+FF";
            }
        } else {
            lastTurnSource = "MANUAL";
        }

        updateAutoAlignRumble(gamepad1, nowMs);
        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void clearTriangleState() {
        relocalizeRequested = false;
        relocalizeStartMs = 0;
        triangleWritebackSeen = false;
        triangleSettledSinceMs = 0;
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
        triangleTimedOut = false;
        clearTriangleState();

        follower.startTeleopDrive();
        follower.setPose(new Pose(45, -120, Math.toRadians(140)));

        if (aimController != null) {
            aimController.reset();
            aimController.setGoal(GOAL_X, GOAL_Y);
            aimController.setRobotPose(45, -120, Math.toRadians(140));
            aimController.setUseVisionCorrection(false);
            aimController.setForceVisionRelocalization(false);
        }

        lastTurnSource = "POSE_RESET";
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;
        lastLimelightAlignPollMs = 0;
        lastLimelightIdlePollMs = 0;
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingAutoTurn) {
        double translationScale;
        double rotationScale;

        if (relocalizeRequested) {
            boolean tagVisible = aimController != null && aimController.isTargetVisible();
            translationScale = tagVisible ? TRIANGLE_TRANSLATION_SCALE : TRIANGLE_TRANSLATION_SCALE_NO_TAG;
            rotationScale = 1.0;
        } else {
            translationScale = slowMode ? NORMAL_SPEED : 1.0;
            rotationScale = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.5);
        }

        lastTranslationScale = translationScale;
        lastRotationScale = rotationScale;

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, false);
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

        telemetry.addData("Triangle Active", relocalizeRequested);
        telemetry.addData("Triangle Writeback Seen", triangleWritebackSeen);
        telemetry.addData("Triangle Settled Since", triangleSettledSinceMs);
        telemetry.addData("Triangle Timed Out", triangleTimedOut);

        telemetry.addData("Turn Source", lastTurnSource);
        telemetry.addData("Auto Turn (Raw)", "%.3f", lastVisionTurn);
        telemetry.addData("Turn (Applied)", "%.3f", lastAppliedTurn);
        telemetry.addData("Drive Input", "%.2f", lastDrive);
        telemetry.addData("Strafe Input", "%.2f", lastStrafe);
        telemetry.addData("Translation Scale", "%.2f", lastTranslationScale);
        telemetry.addData("Rotation Scale", "%.2f", lastRotationScale);
    }
}