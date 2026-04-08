package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GoalAimController {

    private final Telemetry telemetry;
    private final Limelight limelight;
    private final Follower follower;

    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    private double goalX = GOAL_X;
    private double goalY = GOAL_Y;

    private boolean useVisionCorrection = false;
    private boolean forceVisionRelocalization = false;

    private double lastHeadingErrorDeg = 0.0;
    private double turnPower = 0.0;
    private boolean usingFastProfile = false;
    private boolean visionRelocActive = false;

    private long lastUpdateNs = 0;

    private double baseDesiredHeadingDeg = 0.0;
    private double poseOffsetDeg = 0.0;
    private double odomDesiredHeadingDeg = 0.0;
    private double odomErrorDegForGate = 0.0;

    private double headingRelocalizationDeg = 0.0;
    private double relocalizedHeadingDeg = 0.0;
    private double filteredHeadingRelocalizationDeg = 0.0;

    private boolean visionWritebackAppliedThisUpdate = false;
    private boolean visionWritebackAppliedLatched = false;

    private static final double VISION_BLEND_ALPHA = 0.40;
    private static final double MAX_HEADING_RELOCALIZATION_DEG = 12.0;

    private static final double SNAP_GAIN = 0.60;
    private static final double MAX_HEADING_SNAP_STEP_DEG = 4.0;
    private static final double SNAP_DEADBAND_DEG = 0.30;

    private static final double FORCED_SNAP_GAIN = 0.85;
    private static final double MAX_HEADING_SNAP_STEP_DEG_FORCED = 8.0;

    // Renamed: this is not true absolute overwrite, it is a full delta writeback
    private static final boolean FORCED_USE_FULL_DELTA_WRITEBACK = true;

    private static final double MAX_STABLE_FRAME_DELTA_DEG = 0.6;
    private static final int REQUIRED_STABLE_FRAMES = 2;
    private static final int REQUIRED_STABLE_FRAMES_FORCED = 4;

    private static final double MAX_FORCED_ACCEPTED_RAW_RELOC_DEG = 10.0;
    private static final double FORCED_VISION_ENABLE_ODOM_ERROR_DEG = 14.0;

    private static final double HEADING_RATE_ALPHA = 0.25;
    private static final double MAX_RELOCALIZE_RATE_DEG_PER_SEC = 20.0;
    private double filteredHeadingRateDegPerSec = 0.0;
    private double lastRobotHeadingDegForRate = 0.0;
    private boolean haveLastHeadingForRate = false;

    private double lastRawHeadingRelocDeg = 0.0;
    private int stableVisionFrames = 0;

    // V2 state/debug
    private static final long MIN_FORCED_VISIBLE_MS = 80;
    private static final long WRITEBACK_COOLDOWN_MS = 120;
    private long lastWritebackMs = 0;
    private long triangleStartedMs = 0;

    private boolean rejectRateTooHigh = false;
    private boolean rejectOdomDisagreement = false;
    private boolean rejectVisibilityTooShort = false;
    private boolean rejectForcedRawTooLarge = false;

    private double lastConfidence = 0.0;

    public GoalAimController(Follower follower, Limelight limelight, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.limelight = limelight;
    }

    public void setRobotPose(double x, double y, double headingRad) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeadingRad = headingRad;
    }

    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void setTargetBlue() {
        limelight.setTargetBlue();
    }

    public void setTargetRed() {
        limelight.setTargetRed();
    }

    public void setUseVisionCorrection(boolean useVisionCorrection) {
        this.useVisionCorrection = useVisionCorrection;
    }

    public void setForceVisionRelocalization(boolean forceVisionRelocalization) {
        this.forceVisionRelocalization = forceVisionRelocalization;
    }

    public void noteTriangleStart() {
        triangleStartedMs = System.currentTimeMillis();
    }

    public void pollVision() {
        limelight.pollVision();
    }

    public void update() {
        visionWritebackAppliedThisUpdate = false;

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double baseDesiredHeadingRad = Math.atan2(dy, dx) + Math.toRadians(180.0);
        baseDesiredHeadingDeg = Math.toDegrees(baseDesiredHeadingRad);

        poseOffsetDeg = getPoseBasedAimOffsetDeg(robotX, robotY);
        limelight.setTargetAngle(poseOffsetDeg);

        double odomDesiredHeadingRad = baseDesiredHeadingRad + Math.toRadians(poseOffsetDeg);
        odomDesiredHeadingDeg = Math.toDegrees(odomDesiredHeadingRad);

        double odomErrorRad = wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad);
        odomErrorDegForGate = Math.toDegrees(odomErrorRad);

        boolean odomCloseEnoughNormal =
                Math.abs(odomErrorDegForGate) <= VISION_ENABLE_ODOM_ERROR_DEG;

        boolean odomCloseEnoughForced =
                Math.abs(odomErrorDegForGate) <= FORCED_VISION_ENABLE_ODOM_ERROR_DEG;

        boolean odomAgreementOk =
                forceVisionRelocalization ? odomCloseEnoughForced : odomCloseEnoughNormal;

        long nowNs = System.nanoTime();
        double dt = 0.02;
        if (lastUpdateNs != 0) {
            dt = (nowNs - lastUpdateNs) / 1_000_000_000.0;
        }
        lastUpdateNs = nowNs;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.1) dt = 0.1;

        updateHeadingRate(dt);

        boolean robotSteadyEnough =
                Math.abs(filteredHeadingRateDegPerSec) <= MAX_RELOCALIZE_RATE_DEG_PER_SEC;

        long nowMs = System.currentTimeMillis();
        long visibleDurationMs = limelight.isTargetVisible()
                ? (nowMs - limelight.getLastTargetAcquiredMs())
                : 0;

        boolean visibilityOk = !forceVisionRelocalization
                || visibleDurationMs >= MIN_FORCED_VISIBLE_MS;

        boolean allowVisionRelocalization =
                useVisionCorrection
                        && limelight.isTargetVisible()
                        && robotSteadyEnough
                        && odomAgreementOk
                        && visibilityOk;

        rejectRateTooHigh = useVisionCorrection && limelight.isTargetVisible() && !robotSteadyEnough;
        rejectOdomDisagreement = useVisionCorrection && limelight.isTargetVisible() && !odomAgreementOk;
        rejectVisibilityTooShort = useVisionCorrection && limelight.isTargetVisible()
                && forceVisionRelocalization && !visibilityOk;
        rejectForcedRawTooLarge = false;

        visionRelocActive = false;
        lastConfidence = 0.0;

        if (allowVisionRelocalization) {
            if (limelight.isFreshFrameThisLoop()) {
                double rawHeadingRelocDeg = limelight.getHeadingBiasObservationDeg();
                rawHeadingRelocDeg = clamp(
                        rawHeadingRelocDeg,
                        -MAX_HEADING_RELOCALIZATION_DEG,
                        MAX_HEADING_RELOCALIZATION_DEG
                );

                if (forceVisionRelocalization
                        && Math.abs(rawHeadingRelocDeg) > MAX_FORCED_ACCEPTED_RAW_RELOC_DEG) {
                    stableVisionFrames = 0;
                    rejectForcedRawTooLarge = true;
                } else {
                    updateStableVisionState(rawHeadingRelocDeg);

                    filteredHeadingRelocalizationDeg +=
                            VISION_BLEND_ALPHA * (rawHeadingRelocDeg - filteredHeadingRelocalizationDeg);

                    int requiredStableFrames = forceVisionRelocalization
                            ? REQUIRED_STABLE_FRAMES_FORCED
                            : REQUIRED_STABLE_FRAMES;

                    double confidence = 0.0;
                    if (limelight.isTargetVisible()) confidence += 0.25;
                    if (robotSteadyEnough) confidence += 0.25;
                    if (stableVisionFrames >= requiredStableFrames) confidence += 0.25;
                    if (odomAgreementOk) confidence += 0.25;
                    lastConfidence = confidence;

                    if (confidence >= 1.0) {
                        applyHeadingWriteback(forceVisionRelocalization);
                        visionRelocActive = true;
                    }
                }
            }
        } else {
            stableVisionFrames = 0;

            filteredHeadingRelocalizationDeg *= 0.85;
            if (Math.abs(filteredHeadingRelocalizationDeg) < 0.05) {
                filteredHeadingRelocalizationDeg = 0.0;
            }
        }

        headingRelocalizationDeg = filteredHeadingRelocalizationDeg;

        double relocalizedHeadingRad =
                robotHeadingRad + Math.toRadians(headingRelocalizationDeg);
        relocalizedHeadingDeg = Math.toDegrees(relocalizedHeadingRad);

        double errorRad = wrapAngleRad(odomDesiredHeadingRad - relocalizedHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        usingFastProfile = robotY < FAST_AIM_Y_THRESHOLD;

        double kP = usingFastProfile ? FAST_KP_TURN : PRECISE_KP_TURN;
        double kD = usingFastProfile ? FAST_KD_TURN : PRECISE_KD_TURN;
        double kS = usingFastProfile ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
        double deadbandDeg = usingFastProfile ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

        double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double out = 0.0;
        if (Math.abs(errorDeg) > deadbandDeg) {
            out = (kP * errorDeg) + (kD * derivative) + Math.signum(errorDeg) * kS;
        }

        turnPower = clamp(out, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        if (telemetry != null) {
            telemetry.addData("Aim Goal", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addData("Aim Pose", "(%.1f, %.1f, %.1fdeg)",
                    robotX, robotY, Math.toDegrees(robotHeadingRad));

            telemetry.addData("Aim Base Heading", "%.2f", baseDesiredHeadingDeg);
            telemetry.addData("Aim Pose Offset", "%.2f", poseOffsetDeg);

            telemetry.addData("Aim Odom Heading", "%.2f", odomDesiredHeadingDeg);
            telemetry.addData("Aim Odom Err", "%.2f", odomErrorDegForGate);
            telemetry.addData("Aim Vision Gate", odomAgreementOk);
            telemetry.addData("Aim Vision Gate Mode", forceVisionRelocalization ? "FORCED" : "NORMAL");
            telemetry.addData("Aim Vision Thresh", "%.2f",
                    forceVisionRelocalization
                            ? FORCED_VISION_ENABLE_ODOM_ERROR_DEG
                            : VISION_ENABLE_ODOM_ERROR_DEG);
            telemetry.addData("Aim Force Reloc", forceVisionRelocalization);

            telemetry.addData("Aim Rate Deg/S", "%.1f", filteredHeadingRateDegPerSec);
            telemetry.addData("Aim Target Visible", limelight.isTargetVisible());
            telemetry.addData("Aim Visible Ms", visibleDurationMs);
            telemetry.addData("Aim Allow Reloc", allowVisionRelocalization);

            telemetry.addData("Aim Reject Rate", rejectRateTooHigh);
            telemetry.addData("Aim Reject Odom", rejectOdomDisagreement);
            telemetry.addData("Aim Reject Visible", rejectVisibilityTooShort);
            telemetry.addData("Aim Reject Raw Large", rejectForcedRawTooLarge);
            telemetry.addData("Aim Confidence", "%.2f", lastConfidence);

            telemetry.addData("Aim Heading Reloc", "%.2f", headingRelocalizationDeg);
            telemetry.addData("Aim Reloc Heading", "%.2f", relocalizedHeadingDeg);
            telemetry.addData("Aim Err Deg", "%.2f", errorDeg);
            telemetry.addData("Aim Profile", usingFastProfile ? "FAST" : "PRECISE");
            telemetry.addData("Aim Use Vision", useVisionCorrection);
            telemetry.addData("Aim Vision Active", visionRelocActive);
            telemetry.addData("Aim Stable Frames", stableVisionFrames);
            telemetry.addData("Aim Turn", "%.3f", turnPower);
            telemetry.addData("Aim dt", "%.3f", dt);
            telemetry.addData("Aim Writeback", visionWritebackAppliedThisUpdate);
        }
    }

    private void updateHeadingRate(double dt) {
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        if (!haveLastHeadingForRate) {
            lastRobotHeadingDegForRate = robotHeadingDeg;
            haveLastHeadingForRate = true;
            filteredHeadingRateDegPerSec = 0.0;
            return;
        }

        double deltaDeg = wrapAngleDeg(robotHeadingDeg - lastRobotHeadingDegForRate);
        double rawRateDegPerSec = deltaDeg / dt;

        filteredHeadingRateDegPerSec +=
                HEADING_RATE_ALPHA * (rawRateDegPerSec - filteredHeadingRateDegPerSec);

        lastRobotHeadingDegForRate = robotHeadingDeg;
    }

    private void updateStableVisionState(double rawHeadingRelocDeg) {
        if (stableVisionFrames == 0) {
            stableVisionFrames = 1;
        } else {
            if (Math.abs(rawHeadingRelocDeg - lastRawHeadingRelocDeg) <= MAX_STABLE_FRAME_DELTA_DEG) {
                stableVisionFrames++;
            } else {
                stableVisionFrames = 1;
            }
        }

        lastRawHeadingRelocDeg = rawHeadingRelocDeg;
    }

    private void applyHeadingWriteback(boolean forced) {
        if (follower == null) return;

        long nowMs = System.currentTimeMillis();
        if ((nowMs - lastWritebackMs) < WRITEBACK_COOLDOWN_MS) {
            return;
        }

        Pose currentPose = follower.getPose();
        if (currentPose == null) return;

        double correctedHeading;

        if (forced && FORCED_USE_FULL_DELTA_WRITEBACK) {
            correctedHeading = wrapAngleRad(
                    currentPose.getHeading() + Math.toRadians(filteredHeadingRelocalizationDeg)
            );
            filteredHeadingRelocalizationDeg = 0.0;
        } else {
            double gain = forced ? FORCED_SNAP_GAIN : SNAP_GAIN;
            double maxStep = forced ? MAX_HEADING_SNAP_STEP_DEG_FORCED : MAX_HEADING_SNAP_STEP_DEG;

            double snapStepDeg = gain * filteredHeadingRelocalizationDeg;
            snapStepDeg = clamp(snapStepDeg, -maxStep, maxStep);

            if (Math.abs(snapStepDeg) < SNAP_DEADBAND_DEG) {
                return;
            }

            correctedHeading = wrapAngleRad(
                    currentPose.getHeading() + Math.toRadians(snapStepDeg)
            );

            filteredHeadingRelocalizationDeg -= snapStepDeg;
            if (Math.abs(filteredHeadingRelocalizationDeg) < 0.05) {
                filteredHeadingRelocalizationDeg = 0.0;
            }
        }

        follower.setPose(new Pose(
                currentPose.getX(),
                currentPose.getY(),
                correctedHeading
        ));

        robotHeadingRad = correctedHeading;

        visionWritebackAppliedThisUpdate = true;
        visionWritebackAppliedLatched = true;
        stableVisionFrames = 0;
        lastWritebackMs = nowMs;
    }

    public double getTurnPower() {
        return turnPower;
    }

    public boolean isTargetVisible() {
        return limelight.isTargetVisible();
    }

    public double getEffectiveAimErrorDeg() {
        return lastHeadingErrorDeg;
    }

    public boolean isSettled() {
        double shootDeadband = usingFastProfile
                ? FAST_SHOOT_READY_HEADING_ERROR_DEG
                : PRECISE_SHOOT_READY_HEADING_ERROR_DEG;

        double maxTurn = usingFastProfile
                ? FAST_SHOOT_READY_MAX_TURN
                : PRECISE_SHOOT_READY_MAX_TURN;

        return Math.abs(lastHeadingErrorDeg) <= shootDeadband
                && Math.abs(turnPower) <= maxTurn;
    }

    public boolean isShootReady() {
        return isSettled();
    }

    public String getAimProfileName() {
        return usingFastProfile ? "FAST" : "PRECISE";
    }

    public boolean isUsingFastProfile() {
        return usingFastProfile;
    }

    public int getDetectedTagId() {
        return limelight.getDetectedTagId();
    }

    public double getFilteredRate() {
        return filteredHeadingRateDegPerSec;
    }

    public double getLastError() {
        return lastHeadingErrorDeg;
    }

    public double getHeadingErrorDeg() {
        return lastHeadingErrorDeg;
    }

    public boolean isFreshFrameThisLoop() {
        return limelight.isFreshFrameThisLoop();
    }

    public double getVisionTx() {
        return limelight.getTx();
    }

    public double getHeadingRelocalizationDeg() {
        return headingRelocalizationDeg;
    }

    public double getRelocalizedHeadingDeg() {
        return relocalizedHeadingDeg;
    }

    public boolean didApplyVisionWritebackThisUpdate() {
        return visionWritebackAppliedThisUpdate;
    }

    public boolean didApplyVisionWriteback() {
        if (visionWritebackAppliedLatched) {
            visionWritebackAppliedLatched = false;
            return true;
        }
        return false;
    }

    public double getLastConfidence() {
        return lastConfidence;
    }

    public void sendTelemetry() {
        limelight.sendTelemetry();
    }

    public void reset() {
        lastHeadingErrorDeg = 0.0;
        turnPower = 0.0;
        usingFastProfile = false;
        visionRelocActive = false;
        lastUpdateNs = 0;

        baseDesiredHeadingDeg = 0.0;
        poseOffsetDeg = 0.0;
        odomDesiredHeadingDeg = 0.0;
        odomErrorDegForGate = 0.0;

        headingRelocalizationDeg = 0.0;
        relocalizedHeadingDeg = 0.0;
        filteredHeadingRelocalizationDeg = 0.0;

        visionWritebackAppliedThisUpdate = false;
        visionWritebackAppliedLatched = false;

        filteredHeadingRateDegPerSec = 0.0;
        lastRobotHeadingDegForRate = 0.0;
        haveLastHeadingForRate = false;

        lastRawHeadingRelocDeg = 0.0;
        stableVisionFrames = 0;

        lastWritebackMs = 0;
        triangleStartedMs = 0;
        rejectRateTooHigh = false;
        rejectOdomDisagreement = false;
        rejectVisibilityTooShort = false;
        rejectForcedRawTooLarge = false;
        lastConfidence = 0.0;
    }

    private double getPoseBasedAimOffsetDeg(double fieldX, double fieldY) {
        // Keep your existing implementation here
        return 0.0;
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapAngleDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}