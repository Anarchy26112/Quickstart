package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class GoalAimController {
    public enum AllianceColor {
        BLUE,
        RED
    }

    private final Telemetry telemetry;
    private final Limelight limelight;
    private final Follower follower;

    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    private double goalX = 0.0;
    private double goalY = 0.0;
    private AllianceColor allianceColor = AllianceColor.BLUE;

    private boolean useVisionCorrection = false;
    private boolean forceTagCentering = false;

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

    private static final double MAX_HEADING_RELOCALIZATION_DEG = 10.0;

    private static final double SNAP_GAIN = 0.60;
    private static final double MAX_HEADING_SNAP_STEP_DEG = 4.0;
    private static final double SNAP_DEADBAND_DEG = 0.30;

    private static final double FORCED_SNAP_GAIN = 0.85;
    private static final double MAX_HEADING_SNAP_STEP_DEG_FORCED = 8.0;

    private static final double MAX_STABLE_FRAME_DELTA_DEG = 0.8;
    private static final int REQUIRED_STABLE_FRAMES = 2;
    private static final int REQUIRED_STABLE_FRAMES_FORCED = 1;

    private static final double MAX_FORCED_ACCEPTED_RAW_RELOC_DEG = 10.0;
    private static final double FORCED_VISION_ENABLE_ODOM_ERROR_DEG = 18.0;

    private static final double HEADING_RATE_ALPHA = 0.25;
    private static final double MAX_RELOCALIZE_RATE_DEG_PER_SEC = 20.0;
    private double filteredHeadingRateDegPerSec = 0.0;
    private double lastRobotHeadingDegForRate = 0.0;
    private boolean haveLastHeadingForRate = false;

    private double lastRawHeadingRelocDeg = 0.0;
    private int stableVisionFrames = 0;

    private static final long MIN_FORCED_VISIBLE_MS = 40;
    private static final long WRITEBACK_COOLDOWN_MS = 40;
    private long lastWritebackMs = 0;
    private long tagCenterStartedMs = 0;

    private boolean rejectRateTooHigh = false;
    private boolean rejectOdomDisagreement = false;
    private boolean rejectVisibilityTooShort = false;
    private boolean rejectForcedRawTooLarge = false;

    private double lastConfidence = 0.0;

    private static final int FORCED_SAMPLE_WINDOW = 2;
    private static final double FORCED_SAMPLE_MAX_SPREAD_DEG = 0.90;
    private final List<Double> forcedStableSamplesDeg = new ArrayList<>();
    private double acceptedForcedRelocDeg = 0.0;

    private static final double TAG_CENTER_TARGET_TX_DEG = 0.0;

    private static final double TAG_CENTER_TX_ALPHA = 0.65;
    private static final double TAG_CENTER_KD = 0.0020;
    private static final double TAG_CENTER_D_MAX = 0.08;
    private static final double TAG_CENTER_DERIVATIVE_DEADBAND_DEG = 0.10;

    private double filteredTagCenterTxErrDeg = 0.0;
    private double lastTagCenterTxErrDeg = 0.0;

    private static final double NORMAL_MODE_VISION_BLEND_CAP = 0.60;

    private long shootReadySinceMs = 0;
    private boolean shootReadyRaw = false;
    private boolean shootReadyLatched = false;
    private String shootBlockReason = "INIT";

    // NEW: debug breakdown of confidence-weighted blending
    private double lastVisionBlendAlpha = 0.0;
    private double lastVisionConfidenceArea = 0.0;
    private double lastVisionConfidenceFreshness = 0.0;
    private double lastVisionConfidenceStability = 0.0;
    private double lastVisionConfidenceRate = 0.0;
    private double lastVisionConfidenceOdom = 0.0;

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
    public void setAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        if (allianceColor == AllianceColor.RED) {
            limelight.setTargetRed();
            setGoal(GOAL_X, GOAL_Y_RED);
        } else {
            limelight.setTargetBlue();
            setGoal(GOAL_X, GOAL_Y_BLUE);
        }
    }
    private double getAllianceYSign() {
        return (allianceColor == AllianceColor.RED) ? -1.0 : 1.0;
    }

    private double getAllianceNormalizedY(double fieldY) {
        // BLUE: keep negative Y as-is
        // RED: flip positive Y into negative Y space
        return fieldY * getAllianceYSign();
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

    public void setForceTagCentering(boolean enabled) {
        this.forceTagCentering = enabled;

        limelight.setUsePureTagCenteringObservation(enabled);
        limelight.setTagCenterTargetAngle(TAG_CENTER_TARGET_TX_DEG);

        if (!enabled) {
            forcedStableSamplesDeg.clear();
            acceptedForcedRelocDeg = 0.0;
            filteredTagCenterTxErrDeg = 0.0;
            lastTagCenterTxErrDeg = 0.0;
        }
    }

    public void noteTagCenterStart(long nowMs) {
        tagCenterStartedMs = nowMs;
    }

    public void pollVision(long nowMs) {
        limelight.pollVision(nowMs);
    }

    public void reset() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeadingRad = 0.0;

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
        tagCenterStartedMs = 0;

        rejectRateTooHigh = false;
        rejectOdomDisagreement = false;
        rejectVisibilityTooShort = false;
        rejectForcedRawTooLarge = false;

        lastConfidence = 0.0;
        forcedStableSamplesDeg.clear();
        acceptedForcedRelocDeg = 0.0;

        filteredTagCenterTxErrDeg = 0.0;
        lastTagCenterTxErrDeg = 0.0;

        shootReadySinceMs = 0;
        shootReadyRaw = false;
        shootReadyLatched = false;
        shootBlockReason = "RESET";

        lastVisionBlendAlpha = 0.0;
        lastVisionConfidenceArea = 0.0;
        lastVisionConfidenceFreshness = 0.0;
        lastVisionConfidenceStability = 0.0;
        lastVisionConfidenceRate = 0.0;
        lastVisionConfidenceOdom = 0.0;
    }
    public void update(long nowMs, long nowNs) {
        visionWritebackAppliedThisUpdate = false;

        double dt = 0.02;
        if (lastUpdateNs != 0) {
            dt = (nowNs - lastUpdateNs) * 1e-9;
        }
        lastUpdateNs = nowNs;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.1) dt = 0.1;

        OdomAimState odom = updateOdomAim();

        // Use Limelight only when robot is ABOVE the threshold:
        // y > -36  => enable vision
        // y <= -36 => disable vision
        double normalizedY = getAllianceNormalizedY(robotY);
        boolean allowVisionByY = normalizedY > FAST_AIM_Y_THRESHOLD;

        updateHeadingRate(dt);

        boolean robotSteadyEnough =
                Math.abs(filteredHeadingRateDegPerSec) <= MAX_RELOCALIZE_RATE_DEG_PER_SEC;

        long visibleDurationMs = limelight.isTargetVisible()
                ? (nowMs - limelight.getLastTargetAcquiredMs())
                : 0;

        boolean visibilityOk = !forceTagCentering || visibleDurationMs >= MIN_FORCED_VISIBLE_MS;
        boolean controlVisible = limelight.isControlTargetVisible();

        boolean allowVisionRelocalization =
                useVisionCorrection
                        && allowVisionByY
                        && controlVisible
                        && robotSteadyEnough
                        && odom.odomAgreementOk
                        && visibilityOk;

        rejectRateTooHigh = useVisionCorrection && allowVisionByY && controlVisible && !robotSteadyEnough;
        rejectOdomDisagreement = useVisionCorrection && allowVisionByY && controlVisible && !odom.odomAgreementOk;
        rejectVisibilityTooShort = useVisionCorrection && allowVisionByY && controlVisible
                && forceTagCentering && !visibilityOk;
        rejectForcedRawTooLarge = false;

        visionRelocActive = false;
        lastConfidence = 0.0;
        lastVisionBlendAlpha = 0.0;

        // If vision is not allowed by Y, clear/decay vision contribution
        if (!allowVisionByY) {
            stableVisionFrames = 0;
            forcedStableSamplesDeg.clear();
            acceptedForcedRelocDeg = 0.0;
            filteredTagCenterTxErrDeg = 0.0;
            lastTagCenterTxErrDeg = 0.0;

            filteredHeadingRelocalizationDeg *= VISION_BLEND_DECAY_WHEN_LOST;
            if (Math.abs(filteredHeadingRelocalizationDeg) < 0.05) {
                filteredHeadingRelocalizationDeg = 0.0;
            }
        } else if (allowVisionRelocalization && limelight.isFreshFrameThisLoop()) {
            double rawHeadingRelocDeg = limelight.getHeadingBiasObservationDeg();
            rawHeadingRelocDeg = clamp(
                    rawHeadingRelocDeg,
                    -MAX_HEADING_RELOCALIZATION_DEG,
                    MAX_HEADING_RELOCALIZATION_DEG
            );

            if (forceTagCentering && Math.abs(rawHeadingRelocDeg) > MAX_FORCED_ACCEPTED_RAW_RELOC_DEG) {
                stableVisionFrames = 0;
                forcedStableSamplesDeg.clear();
                rejectForcedRawTooLarge = true;
            } else {
                updateStableVisionState(rawHeadingRelocDeg);

                int requiredStableFrames = forceTagCentering
                        ? REQUIRED_STABLE_FRAMES_FORCED
                        : REQUIRED_STABLE_FRAMES;

                double confidence = computeVisionConfidence(
                        controlVisible,
                        robotSteadyEnough,
                        odom.odomAgreementOk,
                        visibilityOk,
                        stableVisionFrames,
                        requiredStableFrames
                );
                lastConfidence = confidence;

                if (forceTagCentering) {
                    if (stableVisionFrames >= requiredStableFrames) {
                        pushForcedSample(rawHeadingRelocDeg);

                        if (isForcedSampleWindowReady()) {
                            acceptedForcedRelocDeg = medianOfForcedWindow();
                            visionRelocActive = true;
                        }
                    }
                } else {
                    double blendAlpha = computeVisionBlendAlpha(confidence);
                    blendAlpha = Math.min(blendAlpha, NORMAL_MODE_VISION_BLEND_CAP);
                    lastVisionBlendAlpha = blendAlpha;

                    if (blendAlpha > 0.0) {
                        filteredHeadingRelocalizationDeg +=
                                blendAlpha * (rawHeadingRelocDeg - filteredHeadingRelocalizationDeg);
                        visionRelocActive = true;
                    }

                    if (confidence >= VISION_CONFIDENCE_MIN_TO_WRITEBACK) {
                        applyHeadingWriteback(nowMs, false, confidence);
                    }
                }
            }
        } else {
            stableVisionFrames = 0;
            forcedStableSamplesDeg.clear();

            if (!forceTagCentering) {
                filteredHeadingRelocalizationDeg *= VISION_BLEND_DECAY_WHEN_LOST;
                if (Math.abs(filteredHeadingRelocalizationDeg) < 0.05) {
                    filteredHeadingRelocalizationDeg = 0.0;
                }
            }
        }

        headingRelocalizationDeg = (forceTagCentering && allowVisionByY)
                ? acceptedForcedRelocDeg
                : filteredHeadingRelocalizationDeg;

        double effectiveHeadingRad =
                robotHeadingRad + Math.toRadians(headingRelocalizationDeg);

        relocalizedHeadingDeg = Math.toDegrees(effectiveHeadingRad);

        double errorRad = wrapAngleRad(odom.odomDesiredHeadingRad - effectiveHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        usingFastProfile = normalizedY < FAST_AIM_Y_THRESHOLD;

        double kP = usingFastProfile ? FAST_KP_TURN : PRECISE_KP_TURN;
        double kD = usingFastProfile ? FAST_KD_TURN : PRECISE_KD_TURN;
        double kS = usingFastProfile ? FAST_kS_VOLTAGE_COMP : PRECISE_kS_VOLTAGE_COMP;
        double deadbandDeg = usingFastProfile ? FAST_ERROR_DEADBAND_DEG : PRECISE_ERROR_DEADBAND_DEG;

        double derivative = (errorDeg - lastHeadingErrorDeg) / dt;
        lastHeadingErrorDeg = errorDeg;

        double odomOut = 0.0;
        if (Math.abs(errorDeg) > deadbandDeg) {
            odomOut = (kP * errorDeg) + (kD * derivative) + Math.signum(errorDeg) * kS;
        }

        // Tag centering only allowed above threshold too
        if (forceTagCentering && allowVisionByY && controlVisible) {
            double txErr = limelight.getTagCenteringErrorDeg();

            filteredTagCenterTxErrDeg += TAG_CENTER_TX_ALPHA * (txErr - filteredTagCenterTxErrDeg);
            double centerDerivative = (filteredTagCenterTxErrDeg - lastTagCenterTxErrDeg) / dt;
            lastTagCenterTxErrDeg = filteredTagCenterTxErrDeg;

            double centerOut = LIMELIGHT_TRIM_kP * filteredTagCenterTxErrDeg;

            if (Math.abs(filteredTagCenterTxErrDeg) > TAG_CENTER_DERIVATIVE_DEADBAND_DEG) {
                double damp = clamp(TAG_CENTER_KD * centerDerivative, -TAG_CENTER_D_MAX, TAG_CENTER_D_MAX);
                centerOut -= damp;
            }

            if (Math.abs(filteredTagCenterTxErrDeg) <= LIMELIGHT_TX_DEADBAND_DEG) {
                centerOut = 0.0;
            }

            turnPower = clamp(centerOut, -LIMELIGHT_TRIM_MAX, LIMELIGHT_TRIM_MAX);
        } else {
            turnPower = clamp(odomOut, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        }

        updateShootReady(nowMs);

        if (telemetry != null) {
            telemetry.addData("Aim Goal", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addData("Aim Pose", "(%.1f, %.1f, %.1fdeg)",
                    robotX, robotY, Math.toDegrees(robotHeadingRad));

            telemetry.addData("Aim Base Heading", "%.2f", baseDesiredHeadingDeg);
            telemetry.addData("Aim Pose Offset", "%.2f", poseOffsetDeg);

            telemetry.addData("Aim Odom Heading", "%.2f", odomDesiredHeadingDeg);
            telemetry.addData("Aim Odom Err", "%.2f", odomErrorDegForGate);
            telemetry.addData("Aim Vision Gate", odom.odomAgreementOk);
            telemetry.addData("Aim Vision Gate Mode", forceTagCentering ? "TAG_CENTER" : "NORMAL");
            telemetry.addData("Aim Vision Thresh", "%.2f",
                    forceTagCentering
                            ? FORCED_VISION_ENABLE_ODOM_ERROR_DEG
                            : VISION_ENABLE_ODOM_ERROR_DEG);
            telemetry.addData("Aim Force Center", forceTagCentering);
            telemetry.addData("Aim Vision Enabled By Y", allowVisionByY);

            telemetry.addData("Aim Heading Rate Deg/S", "%.1f", filteredHeadingRateDegPerSec);
            telemetry.addData("Aim Target Visible", limelight.isTargetVisible());
            telemetry.addData("Aim Control Visible", controlVisible);
            telemetry.addData("Aim Visible Ms", visibleDurationMs);
            telemetry.addData("Aim Allow Reloc", allowVisionRelocalization);

            telemetry.addData("Aim Reject Rate", rejectRateTooHigh);
            telemetry.addData("Aim Reject Odom", rejectOdomDisagreement);
            telemetry.addData("Aim Reject Visible", rejectVisibilityTooShort);
            telemetry.addData("Aim Reject Raw Large", rejectForcedRawTooLarge);
            telemetry.addData("Aim Confidence", "%.2f", lastConfidence);
            telemetry.addData("Aim Vision Blend Alpha", "%.3f", lastVisionBlendAlpha);
            telemetry.addData("Aim Conf Area", "%.2f", lastVisionConfidenceArea);
            telemetry.addData("Aim Conf Fresh", "%.2f", lastVisionConfidenceFreshness);
            telemetry.addData("Aim Conf Stable", "%.2f", lastVisionConfidenceStability);
            telemetry.addData("Aim Conf Rate", "%.2f", lastVisionConfidenceRate);
            telemetry.addData("Aim Conf Odom", "%.2f", lastVisionConfidenceOdom);

            telemetry.addData("Aim Forced Samples", forcedStableSamplesDeg.size());
            telemetry.addData("Aim Accepted Forced", "%.2f", acceptedForcedRelocDeg);
            telemetry.addData("Aim Center Tx Filtered", "%.2f", filteredTagCenterTxErrDeg);

            telemetry.addData("Aim Heading Reloc", "%.2f", headingRelocalizationDeg);
            telemetry.addData("Aim Reloc Heading", "%.2f", relocalizedHeadingDeg);
            telemetry.addData("Aim Err Deg", "%.2f", errorDeg);
            telemetry.addData("Aim Tag Centering Err", "%.2f", getTagCenteringErrorDeg());
            telemetry.addData("Aim Profile", usingFastProfile ? "FAST" : "PRECISE");
            telemetry.addData("Aim Use Vision", useVisionCorrection);
            telemetry.addData("Aim Vision Active", visionRelocActive);
            telemetry.addData("Aim Stable Frames", stableVisionFrames);
            telemetry.addData("Aim Turn", "%.3f", turnPower);
            telemetry.addData("Aim dt", "%.3f", dt);
            telemetry.addData("Aim Writeback", visionWritebackAppliedThisUpdate);

            telemetry.addData("Shoot Ready Raw", shootReadyRaw);
            telemetry.addData("Shoot Ready Latched", shootReadyLatched);
            telemetry.addData("Shoot Block", shootBlockReason);
        }
    }

    private OdomAimState updateOdomAim() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double baseDesiredHeadingRad = Math.atan2(dy, dx) + Math.PI;
        baseDesiredHeadingDeg = Math.toDegrees(baseDesiredHeadingRad);

        double poseOffsetRad = Math.toRadians(getPoseBasedAimOffsetDeg(robotX, robotY));
        poseOffsetDeg = Math.toDegrees(poseOffsetRad);

        limelight.setTargetAngle(poseOffsetDeg);

        double odomDesiredHeadingRad = baseDesiredHeadingRad + poseOffsetRad;
        odomDesiredHeadingDeg = Math.toDegrees(odomDesiredHeadingRad);

        double odomErrorRad = wrapAngleRad(odomDesiredHeadingRad - robotHeadingRad);
        odomErrorDegForGate = Math.toDegrees(odomErrorRad);

        boolean odomCloseEnoughNormal =
                Math.abs(odomErrorDegForGate) <= VISION_ENABLE_ODOM_ERROR_DEG;

        boolean odomCloseEnoughForced =
                Math.abs(odomErrorDegForGate) <= FORCED_VISION_ENABLE_ODOM_ERROR_DEG;

        boolean odomAgreementOk =
                forceTagCentering ? odomCloseEnoughForced : odomCloseEnoughNormal;

        return new OdomAimState(odomDesiredHeadingRad, odomAgreementOk);
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

    private double computeVisionConfidence(
            boolean controlVisible,
            boolean robotSteadyEnough,
            boolean odomAgreementOk,
            boolean visibilityOk,
            int stableFrames,
            int requiredStableFrames
    ) {
        if (!controlVisible || !visibilityOk) {
            lastVisionConfidenceArea = 0.0;
            lastVisionConfidenceFreshness = 0.0;
            lastVisionConfidenceStability = 0.0;
            lastVisionConfidenceRate = 0.0;
            lastVisionConfidenceOdom = 0.0;
            return 0.0;
        }

        double area = limelight.getTargetArea();
        double frameAgeMs = limelight.getControlFrameAgeMs();

        double areaScore = remapClamp(
                area,
                VISION_CONFIDENCE_MIN_TAG_AREA,
                VISION_CONFIDENCE_GOOD_TAG_AREA,
                0.0,
                1.0
        );

        double freshnessScore = remapClamp(frameAgeMs, 0.0, 55.0, 1.0, 0.0);

        double stabilityScore = clamp(
                (double) stableFrames / Math.max(1, requiredStableFrames),
                0.0,
                1.0
        );

        double rateScore = robotSteadyEnough ? 1.0 : 0.0;
        double odomScore = odomAgreementOk ? 1.0 : 0.0;

        lastVisionConfidenceArea = areaScore;
        lastVisionConfidenceFreshness = freshnessScore;
        lastVisionConfidenceStability = stabilityScore;
        lastVisionConfidenceRate = rateScore;
        lastVisionConfidenceOdom = odomScore;

        return clamp(
                0.25 * areaScore +
                        0.20 * freshnessScore +
                        0.20 * stabilityScore +
                        0.15 * rateScore +
                        0.20 * odomScore,
                0.0,
                1.0
        );
    }

    private double computeVisionBlendAlpha(double confidence) {
        if (confidence <= VISION_CONFIDENCE_MIN_TO_BLEND) return 0.0;

        double t = remapClamp(
                confidence,
                VISION_CONFIDENCE_MIN_TO_BLEND,
                1.0,
                0.0,
                1.0
        );

        return lerp(VISION_BLEND_ALPHA_MIN, VISION_BLEND_ALPHA_MAX, t);
    }

    private void updateStableVisionState(double rawHeadingRelocDeg) {
        double delta = Math.abs(rawHeadingRelocDeg - lastRawHeadingRelocDeg);

        if (stableVisionFrames == 0 || delta <= MAX_STABLE_FRAME_DELTA_DEG) {
            stableVisionFrames++;
        } else {
            stableVisionFrames = 1;
            forcedStableSamplesDeg.clear();
        }

        lastRawHeadingRelocDeg = rawHeadingRelocDeg;
    }

    private void pushForcedSample(double sampleDeg) {
        forcedStableSamplesDeg.add(sampleDeg);
        while (forcedStableSamplesDeg.size() > FORCED_SAMPLE_WINDOW) {
            forcedStableSamplesDeg.remove(0);
        }
    }

    private boolean isForcedSampleWindowReady() {
        if (forcedStableSamplesDeg.size() < FORCED_SAMPLE_WINDOW) return false;

        double a = forcedStableSamplesDeg.get(0);
        double b = forcedStableSamplesDeg.get(1);
        double min = Math.min(a, b);
        double max = Math.max(a, b);
        return (max - min) <= FORCED_SAMPLE_MAX_SPREAD_DEG;
    }

    private void applyHeadingWriteback(long nowMs, boolean forced, double confidence) {
        if (!forced && (nowMs - lastWritebackMs) < WRITEBACK_COOLDOWN_MS) {
            return;
        }

        double rawDeltaDeg = filteredHeadingRelocalizationDeg;
        double deadband = SNAP_DEADBAND_DEG;
        double baseGain = forced ? FORCED_SNAP_GAIN : SNAP_GAIN;
        double maxStep = forced ? MAX_HEADING_SNAP_STEP_DEG_FORCED : MAX_HEADING_SNAP_STEP_DEG;

        if (Math.abs(rawDeltaDeg) <= deadband) {
            return;
        }

        double confidenceScale = forced ? 1.0 : clamp(confidence, 0.0, 1.0);
        double gain = baseGain * confidenceScale;

        double snappedDeltaDeg = clamp(gain * rawDeltaDeg, -maxStep, maxStep);

        filteredHeadingRelocalizationDeg -= snappedDeltaDeg;

        if (Math.abs(filteredHeadingRelocalizationDeg) < 0.05) {
            filteredHeadingRelocalizationDeg = 0.0;
        }

        visionWritebackAppliedThisUpdate = true;
        visionWritebackAppliedLatched = true;
        lastWritebackMs = nowMs;
    }

    private void updateShootReady(long nowMs) {
        double allowedHeadingError = usingFastProfile
                ? FAST_SHOOT_READY_HEADING_ERROR_DEG
                : PRECISE_SHOOT_READY_HEADING_ERROR_DEG;

        double allowedTurn = usingFastProfile
                ? FAST_SHOOT_READY_MAX_TURN
                : PRECISE_SHOOT_READY_MAX_TURN;

        double allowedHeadingRate = usingFastProfile
                ? FAST_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC
                : PRECISE_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC;

        shootReadyRaw =
                Math.abs(lastHeadingErrorDeg) <= allowedHeadingError
                        && Math.abs(turnPower) <= allowedTurn
                        && Math.abs(filteredHeadingRateDegPerSec) <= allowedHeadingRate;

        if (shootReadyRaw) {
            if (shootReadySinceMs == 0) {
                shootReadySinceMs = nowMs;
            }
            shootReadyLatched = (nowMs - shootReadySinceMs) >= SHOOT_READY_SETTLE_MS;
            shootBlockReason = shootReadyLatched ? "READY" : "SETTLING";
        } else {
            shootReadySinceMs = 0;
            shootReadyLatched = false;

            if (Math.abs(lastHeadingErrorDeg) > allowedHeadingError) {
                shootBlockReason = "HEADING";
            } else if (Math.abs(turnPower) > allowedTurn) {
                shootBlockReason = "TURN";
            } else {
                shootBlockReason = "RATE";
            }
        }
    }

    private double getPoseBasedAimOffsetDeg(double fieldX, double fieldY) {
        double y = getAllianceNormalizedY(fieldY);

        // Work in BLUE-style negative-Y space for both alliances
        y = Math.min(y, -36.0); // clamp upper bound

        double offsetDeg;
        if (y <= -108.0) {
            offsetDeg = Y_AIM_OFFSET_FAR_DEG;
        } else if (y <= -60.0) {
            offsetDeg = lerp(Y_AIM_OFFSET_FAR_DEG, Y_AIM_OFFSET_MID_DEG, (y + 108.0) / 48.0);
        } else {
            offsetDeg = lerp(Y_AIM_OFFSET_MID_DEG, Y_AIM_OFFSET_NEAR_DEG, (y + 60.0) / 24.0);
        }

        // BLUE offsets stay positive
        // RED offsets become negative
        return (allianceColor == AllianceColor.RED) ? -offsetDeg : offsetDeg;
    }
    public boolean isShootReady() {
        return shootReadyRaw;
    }
    public boolean isShootReadyLatched() {
        return shootReadyLatched;
    }
    public String getShootBlockReason() {
        return shootBlockReason;
    }

    public double getTurnPower() {
        return turnPower;
    }

    public double getTagCenteringErrorDeg() {
        return limelight.getTagCenteringErrorDeg();
    }

    public boolean isControlTargetVisible() {
        return limelight.isControlTargetVisible();
    }

    public boolean isTargetVisible() {
        return limelight.isTargetVisible();
    }

    public boolean isVisionRelocActive() {
        return visionRelocActive;
    }

    public double getHeadingRelocalizationDeg() {
        return headingRelocalizationDeg;
    }

    public double getFilteredHeadingRateDegPerSec() {
        return filteredHeadingRateDegPerSec;
    }

    private double medianOfForcedWindow() {
        double a = forcedStableSamplesDeg.get(0);
        double b = forcedStableSamplesDeg.get(1);
        return 0.5 * (a + b);
    }

    private static double median(List<Double> values) {
        if (values == null || values.isEmpty()) return 0.0;

        List<Double> copy = new ArrayList<>(values);
        Collections.sort(copy);

        int n = copy.size();
        if ((n & 1) == 1) {
            return copy.get(n / 2);
        }
        return 0.5 * (copy.get(n / 2 - 1) + copy.get(n / 2));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * clamp(t, 0.0, 1.0);
    }

    private static double remapClamp(double x, double inMin, double inMax, double outMin, double outMax) {
        if (inMax <= inMin) return outMin;
        double t = (x - inMin) / (inMax - inMin);
        t = clamp(t, 0.0, 1.0);
        return outMin + (outMax - outMin) * t;
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

    private static class OdomAimState {
        final double odomDesiredHeadingRad;
        final boolean odomAgreementOk;

        OdomAimState(double odomDesiredHeadingRad, boolean odomAgreementOk) {
            this.odomDesiredHeadingRad = odomDesiredHeadingRad;
            this.odomAgreementOk = odomAgreementOk;
        }
    }
}