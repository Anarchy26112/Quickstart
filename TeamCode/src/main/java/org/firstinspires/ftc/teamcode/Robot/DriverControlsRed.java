package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriverControlsRed {

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

    private final Pose parkingRed = new Pose(-25, -30, 0);
    private PathChain parkingRedPath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    // One-shot Triangle = tag-centering request
    private boolean tagCenterRequested = false;

    // Tag-centering state
    private long tagCenterSettledSinceMs = 0;
    private boolean tagCenterTimedOut = false;

    // Cached values for telemetry/debug
    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    // Intake aim (mirror of Blue)
    private static final double INTAKE_AIM_TARGET_DEG = 26.0;
    private static final double INTAKE_AIM_TARGET_RAD = Math.toRadians(INTAKE_AIM_TARGET_DEG);
    private static final double INTAKE_AIM_MAX_TURN = 0.35;
    private static final double INTAKE_AIM_DEADBAND_RAD = Math.toRadians(1.0);

    // Limelight polling cadence
    private static final long LIMELIGHT_IDLE_POLL_MS = 40;
    private static final long LIMELIGHT_ALIGN_POLL_MS = 10;
    private long lastLimelightIdlePollMs = 0;
    private long lastLimelightAlignPollMs = 0;

    // Tag-centering timeout
    private static final long TAG_CENTER_TIMEOUT_MS = 1800;
    private long tagCenterStartMs = 0;

    // End tag-centering once centering is truly settled
    private static final double TAG_CENTER_FINISH_TX_ERROR_DEG = 0.40;
    private static final double TAG_CENTER_FINISH_TURN_POWER = 0.035;
    private static final long TAG_CENTER_SETTLE_HOLD_MS = 100;

    // Translation slow-down during tag-centering
    private static final double TAG_CENTER_TRANSLATION_SCALE = 0.35;
    private static final double TAG_CENTER_TRANSLATION_SCALE_NO_TAG = 0.20;

    private enum RumbleMode { OFF, FAST_PULSE }
    private RumbleMode rumbleMode = RumbleMode.OFF;

    private final RumbleEffect fastPulseEffect;
    private long nextPulseAllowedMs = 0;

    private static final int PULSE_INTERVAL_MS = 250;

    public DriverControlsRed(Follower follower, Telemetry telemetry, GoalAimController aimController) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimController = aimController;

        if (this.aimController != null) {
            this.aimController.setAlliance(GoalAimController.AllianceColor.RED);
        }

        buildParkingPath();

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

    public boolean isTagCenterRequested() {
        return tagCenterRequested;
    }

    public boolean isShootReady() {
        return aimController != null && aimController.isShootReady();
    }

    public boolean isShootReadyLatched() {
        return aimController != null && aimController.isShootReadyLatched();
    }

    public String getShootBlockReason() {
        return aimController != null ? aimController.getShootBlockReason() : "NO_AIM_CONTROLLER";
    }

    public void forceEnableAutoAlign() {
        autoAlignEnabled = true;
        lastLimelightAlignPollMs = 0;
        lastLimelightIdlePollMs = 0;
    }

    public void forceDisableAutoAlign() {
        autoAlignEnabled = false;
        clearTagCenterState();

        if (aimController != null) {
            aimController.setUseVisionCorrection(false);
            aimController.setForceTagCentering(false);
        }
    }

    public void update(Gamepad gamepad1, Pose pose, long nowMs) {
        if (pose == null) return;

        if (aimController != null) {
            aimController.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());
        }

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;

            if (!autoAlignEnabled && aimController != null) {
                aimController.setUseVisionCorrection(tagCenterRequested);
                aimController.setForceTagCentering(tagCenterRequested);
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
                follower.followPath(parkingRedPath);
                homingPathStarted = true;
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (btnTriangle.wasPressed(gamepad1.triangle)) {
            tagCenterRequested = true;
            tagCenterStartMs = nowMs;
            tagCenterSettledSinceMs = 0;
            tagCenterTimedOut = false;

            lastLimelightAlignPollMs = 0;
            lastLimelightIdlePollMs = 0;

            if (aimController != null) {
                aimController.setUseVisionCorrection(true);
                aimController.setForceTagCentering(true);
                aimController.noteTagCenterStart(nowMs);
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

        boolean forceTagCenterActive = tagCenterRequested;

        if (aimController != null) {
            if (forceTagCenterActive) {
                if (nowMs - lastLimelightAlignPollMs >= LIMELIGHT_ALIGN_POLL_MS) {
                    aimController.pollVision(nowMs);
                    lastLimelightAlignPollMs = nowMs;
                }
            } else if (nowMs - lastLimelightIdlePollMs >= LIMELIGHT_IDLE_POLL_MS) {
                aimController.pollVision(nowMs);
                lastLimelightIdlePollMs = nowMs;
            }

            aimController.setUseVisionCorrection(autoAlignEnabled || forceTagCenterActive);
            aimController.setForceTagCentering(forceTagCenterActive);
            long nowNs = System.nanoTime();
            aimController.update(nowMs, nowNs);

            boolean controlVisible = aimController.isControlTargetVisible();
            double centerTxErr = Math.abs(aimController.getTagCenteringErrorDeg());
            boolean snapFinished =
                    controlVisible
                            && centerTxErr <= TAG_CENTER_FINISH_TX_ERROR_DEG
                            && Math.abs(aimController.getTurnPower()) <= TAG_CENTER_FINISH_TURN_POWER;

            if (tagCenterRequested) {
                if (snapFinished) {
                    if (tagCenterSettledSinceMs == 0) {
                        tagCenterSettledSinceMs = nowMs;
                    } else if ((nowMs - tagCenterSettledSinceMs) >= TAG_CENTER_SETTLE_HOLD_MS) {
                        clearTagCenterState();
                        aimController.setUseVisionCorrection(autoAlignEnabled);
                        aimController.setForceTagCentering(false);
                    }
                } else {
                    tagCenterSettledSinceMs = 0;
                }
            }

            if (tagCenterRequested && tagCenterStartMs > 0
                    && (nowMs - tagCenterStartMs) >= TAG_CENTER_TIMEOUT_MS) {
                tagCenterTimedOut = true;
                clearTagCenterState();
                aimController.setUseVisionCorrection(autoAlignEnabled);
                aimController.setForceTagCentering(false);
            }
        }

        boolean snapHeadingAssistActive = autoAlignEnabled || forceTagCenterActive;

        if (forceTagCenterActive) {
            turn = 0.0;
        }

        if (snapHeadingAssistActive && aimController != null) {
            double autoTurn = aimController.getTurnPower();
            autoTurn = clamp(autoTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            lastVisionTurn = autoTurn;
            turn = autoTurn;
            usingAutoTurn = true;

            if (forceTagCenterActive && aimController.isControlTargetVisible()) {
                boolean settled = tagCenterSettledSinceMs != 0;
                lastTurnSource = settled ? "TAG_CENTER_SETTLING" : "TAG_CENTERING";
            } else if (forceTagCenterActive) {
                lastTurnSource = "TAG_CENTER_WAITING_FOR_TAG";
            } else {
                lastTurnSource = "ODOM_PD+VISION_BLEND";
            }
        } else {
            lastTurnSource = "MANUAL";
        }

        updateAutoAlignRumble(gamepad1, nowMs);
        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void clearTagCenterState() {
        tagCenterRequested = false;
        tagCenterStartMs = 0;
        tagCenterSettledSinceMs = 0;
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
        tagCenterTimedOut = false;
        clearTagCenterState();

        follower.startTeleopDrive();
        follower.setPose(new Pose(45, 120, Math.toRadians(-140)));

        if (aimController != null) {
            aimController.reset();
            aimController.setAlliance(GoalAimController.AllianceColor.RED);
            aimController.setRobotPose(45, 120, Math.toRadians(-140));
            aimController.setUseVisionCorrection(false);
            aimController.setForceTagCentering(false);
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

        if (tagCenterRequested) {
            boolean tagVisible = aimController != null && aimController.isControlTargetVisible();
            translationScale = tagVisible ? TAG_CENTER_TRANSLATION_SCALE : TAG_CENTER_TRANSLATION_SCALE_NO_TAG;
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
        Pose startPose = new Pose(45, 120, Math.toRadians(-140));
        parkingRedPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkingRed))
                .build();
    }

    public void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("DC Slow Mode", slowMode);
        telemetry.addData("DC Auto Align", autoAlignEnabled);
        telemetry.addData("DC Intaking", intakingActive);

        telemetry.addData("DC Tag Center Active", tagCenterRequested);
        telemetry.addData("DC Tag Center Timed Out", tagCenterTimedOut);
        telemetry.addData("DC Tag Center Settled Since", tagCenterSettledSinceMs);

        telemetry.addData("DC Last Turn Source", lastTurnSource);
        telemetry.addData("DC Last Vision Turn", "%.3f", lastVisionTurn);
        telemetry.addData("DC Last Applied Turn", "%.3f", lastAppliedTurn);
        telemetry.addData("DC Last Drive", "%.3f", lastDrive);
        telemetry.addData("DC Last Strafe", "%.3f", lastStrafe);

        telemetry.addData("DC Translation Scale", "%.2f", lastTranslationScale);
        telemetry.addData("DC Rotation Scale", "%.2f", lastRotationScale);
        telemetry.addData("DC Homing", homingMechanismEngaged);
        telemetry.addData("DC Homing Path Started", homingPathStarted);

        if (aimController != null) {
            telemetry.addData("DC Tag Center Tx Err", "%.2f", aimController.getTagCenteringErrorDeg());
            telemetry.addData("DC Tag Center Control Visible", aimController.isControlTargetVisible());
            telemetry.addData("DC Shoot Ready Raw", aimController.isShootReady());
            telemetry.addData("DC Shoot Ready Latched", aimController.isShootReadyLatched());
            telemetry.addData("DC Shoot Block", aimController.getShootBlockReason());
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

    // ---------- Backward-compatible wrappers ----------
    public boolean isTriangleActive() {
        return tagCenterRequested;
    }
}