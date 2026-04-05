package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight {
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    // =========================
    // CAMERA CONFIGURATION
    // =========================
    private static final double CAMERA_TILT_DEGREES = 0.0;
    private static final double CAMERA_HEIGHT_INCHES = 10.25;
    private static final double APRILTAG_HEIGHT_INCHES = 29.5;

    // =========================
    // VISION STATE
    // =========================
    private LLResult result;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double horizontalDistance = 0.0;
    private double angleToTarget = 0.0;
    private long lastCaptureTimeMs = 0;
    private boolean freshFrameThisLoop = false;

    // =========================
    // PD CONTROL STATE
    // =========================
    private double desiredTx = 0.0;
    private double currentTurnPower = 0.0;
    private double lastKnownTurnPower = 0.0;

    private boolean derivativeInitialized = false;
    private double previousMeasurement = 0.0;
    private double filteredRate = 0.0;

    private final ElapsedTime controlLoopTimer = new ElapsedTime();
    private final ElapsedTime freshFrameTimer = new ElapsedTime();

    // Debug telemetry
    private double lastPTerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastFeedForward = 0.0;
    private double lastError = 0.0;
    private double lastVisionDt = 0.0;
    private double lastControlDt = 0.0;
    private double lastRawRate = 0.0;
    private double lastRawPower = 0.0;

    // =========================
    // TARGET FILTERING & LOSS
    // =========================
    private final List<Integer> allowedTagIds = new ArrayList<>();
    private int previousDetectedTagId = -1;
    private final ElapsedTime targetLossTimer = new ElapsedTime();
    private boolean hadTargetPreviously = false;

    private static final double TARGET_LOST_HOLD_SECONDS = 0.10;
    private static final double TARGET_LOST_RESET_SECONDS = 0.45;
    private static final double REACQUIRE_RESET_SECONDS = 0.06;
    private static final double STALE_FRAME_TIMEOUT_SECONDS = 0.09;

    // =========================
    // CONTROL TUNING
    // =========================
    private static final double DERIVATIVE_FILTER_TC = 0.012;

    // Vision / control timing guards
    private static final double MIN_VALID_VISION_DT = 0.008;
    private static final double MAX_VALID_VISION_DT = 0.22;
    private static final double MAX_CONTROL_DT = 0.05;

    // Rate limits
    private static final double MAX_RAW_RATE_DEG_PER_SEC = 420.0;
    private static final double MAX_FILTERED_RATE_DEG_PER_SEC = 150.0;

    // Output limits
    private static final double MAX_TURN_POWER = 0.25;
    private static final double MAX_D_TERM_POWER = 0.25;

    // Slew rates
    private static final double MAX_POWER_ACCEL_PER_SEC = 4.0;
    private static final double MAX_POWER_DECEL_PER_SEC = 6.0;
    private static final double MAX_POWER_REVERSE_PER_SEC = 5.0;

    // D-term freshness window
    private static final double D_FADE_START_SECONDS = 0.020;
    private static final double D_FADE_END_SECONDS = 0.055;

    // Target loss hold
    private static final double TARGET_LOSS_HOLD_POWER_SCALE = 0.22;
    private static final double TARGET_LOSS_MAX_HOLD_POWER = 0.08;

    // Large setpoint step threshold
    private static final double LARGE_SETPOINT_STEP_DEG = 2.0;

    // =========================
    // FIELD-POSITION PROFILE
    // =========================
    private double robotY = -120.0;
    private boolean useFastAimProfile = true;

    private double activeKpTurn = Kp_TURN;
    private double activeKdTurn = Kd_TURN;
    private double activeKsTurn = PRECISE_kS_VOLTAGE_COMP;
    private double activeErrorDeadbandDeg = 0.6;

    private double activeSettleEnterDeadbandDeg = 1.0;
    private double activeSettleExitDeadbandDeg = 1.2;
    private double activeSettleEnterRateDps = 2.5;
    private double activeSettleExitRateDps = 4.5;

    private double activeShootReadyEnterDeadbandDeg = 3.5;
    private double activeShootReadyExitDeadbandDeg = 4.2;
    private double activeShootReadyEnterRateDps = 10.0;
    private double activeShootReadyExitRateDps = 15.0;

    private boolean settled = false;
    private boolean shootReady = false;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        this.limelight.pipelineSwitch(4);
        this.limelight.start();

        targetLossTimer.reset();
        controlLoopTimer.reset();
        freshFrameTimer.reset();

        updateAimProfile();
    }

    // =========================
    // TARGET CONFIGURATION
    // =========================
    public void setTargetBlue() { setAllowedTags(20); }
    public void setTargetRed() { setAllowedTags(24); }
    public void setTargetMotif() { setAllowedTags(21, 22, 23); }
    public void trackAnyTag() { allowedTagIds.clear(); }

    private void setAllowedTags(Integer... tags) {
        allowedTagIds.clear();
        allowedTagIds.addAll(Arrays.asList(tags));
    }

    // =========================
    // Y-BASED AIM PROFILE
    // =========================
    public void setRobotY(double robotY) {
        this.robotY = robotY;
        updateAimProfile();
    }

    private void updateAimProfile() {
        boolean newFastProfile = robotY < FAST_AIM_Y_THRESHOLD;

        if (newFastProfile != useFastAimProfile) {
            settled = false;
            shootReady = false;
        }

        useFastAimProfile = newFastProfile;

        if (useFastAimProfile) {
            activeKpTurn = FAST_KP_TURN;
            activeKdTurn = FAST_KD_TURN;
            activeErrorDeadbandDeg = FAST_ERROR_DEADBAND_DEG;
            activeKsTurn = FAST_kS_VOLTAGE_COMP;

            activeSettleEnterDeadbandDeg = FAST_SETTLE_ENTER_DEADBAND_DEG;
            activeSettleExitDeadbandDeg = FAST_SETTLE_EXIT_DEADBAND_DEG;
            activeSettleEnterRateDps = FAST_SETTLE_ENTER_RATE_DPS;
            activeSettleExitRateDps = FAST_SETTLE_EXIT_RATE_DPS;

            activeShootReadyEnterDeadbandDeg = FAST_SHOOT_READY_ENTER_DEADBAND_DEG;
            activeShootReadyExitDeadbandDeg = FAST_SHOOT_READY_EXIT_DEADBAND_DEG;
            activeShootReadyEnterRateDps = FAST_SHOOT_READY_ENTER_RATE_DPS;
            activeShootReadyExitRateDps = FAST_SHOOT_READY_EXIT_RATE_DPS;
        } else {
            activeKpTurn = PRECISE_KP_TURN;
            activeKdTurn = PRECISE_KD_TURN;
            activeErrorDeadbandDeg = PRECISE_ERROR_DEADBAND_DEG;
            activeKsTurn = PRECISE_kS_VOLTAGE_COMP;

            activeSettleEnterDeadbandDeg = PRECISE_SETTLE_ENTER_DEADBAND_DEG;
            activeSettleExitDeadbandDeg = PRECISE_SETTLE_EXIT_DEADBAND_DEG;
            activeSettleEnterRateDps = PRECISE_SETTLE_ENTER_RATE_DPS;
            activeSettleExitRateDps = PRECISE_SETTLE_EXIT_RATE_DPS;

            activeShootReadyEnterDeadbandDeg = PRECISE_SHOOT_READY_ENTER_DEADBAND_DEG;
            activeShootReadyExitDeadbandDeg = PRECISE_SHOOT_READY_EXIT_DEADBAND_DEG;
            activeShootReadyEnterRateDps = PRECISE_SHOOT_READY_ENTER_RATE_DPS;
            activeShootReadyExitRateDps = PRECISE_SHOOT_READY_EXIT_RATE_DPS;
        }
    }

    /**
     * Set desired heading offset (degrees). Large changes reset derivative.
     */
    public void setTargetAngle(double angleDegrees) {
        if (Math.abs(angleDegrees - desiredTx) > 1e-6) {
            boolean largeStep = Math.abs(angleDegrees - desiredTx) >= LARGE_SETPOINT_STEP_DEG;
            desiredTx = angleDegrees;
            settled = false;
            shootReady = false;

            if (largeStep) {
                derivativeInitialized = false;
                filteredRate = 0.0;
                lastRawRate = 0.0;
            }
        }
    }

    public void update() {
        pollVision();
        updateControl();
    }

    public boolean pollVision() {
        result = limelight.getLatestResult();
        return processVisionResult(result);
    }

    public void updateControl() {
        calculatePD();
    }
    public void refreshTunables() {
        updateAimProfile();
    }

    // =========================
    // VISION PROCESSING
    // =========================
    private boolean processVisionResult(LLResult result) {
        freshFrameThisLoop = false;
        boolean wasVisibleLastLoop = targetVisible;

        if (result == null || !result.isValid()) {
            if (freshFrameTimer.seconds() > STALE_FRAME_TIMEOUT_SECONDS) {
                markTargetNotVisible();
            }
            return false;
        }

        long captureTimeMs = result.getControlHubTimeStamp();

        if (lastCaptureTimeMs != 0 && captureTimeMs == lastCaptureTimeMs) {
            if (freshFrameTimer.seconds() > STALE_FRAME_TIMEOUT_SECONDS) {
                markTargetNotVisible();
            }
            return false;
        }

        LLResultTypes.FiducialResult bestTag = null;
        double maxArea = -1.0;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        for (int i = 0; i < tags.size(); i++) {
            LLResultTypes.FiducialResult tag = tags.get(i);
            int id = (int) tag.getFiducialId();

            if (allowedTagIds.isEmpty() || allowedTagIds.contains(id)) {
                if (tag.getTargetArea() > maxArea) {
                    maxArea = tag.getTargetArea();
                    bestTag = tag;
                }
            }
        }

        if (bestTag != null) {
            tx = bestTag.getTargetXDegrees();
            ty = bestTag.getTargetYDegrees();
            ta = bestTag.getTargetArea();
            detectedTagId = (int) bestTag.getFiducialId();

            calculateDistanceAndAngle();

            freshFrameThisLoop = true;
            freshFrameTimer.reset();

            boolean reacquireReset =
                    !wasVisibleLastLoop && targetLossTimer.seconds() > REACQUIRE_RESET_SECONDS;
            boolean tagSwitchReset =
                    previousDetectedTagId != -1 && detectedTagId != previousDetectedTagId;

            if (reacquireReset || tagSwitchReset) {
                resetDerivativeState(captureTimeMs);
                settled = false;
                shootReady = false;
            } else {
                updateDerivative(captureTimeMs);
            }

            lastCaptureTimeMs = captureTimeMs;
            targetVisible = true;
            previousDetectedTagId = detectedTagId;
            targetLossTimer.reset();
            hadTargetPreviously = true;
        } else {
            markTargetNotVisible();
        }

        if (freshFrameTimer.seconds() > STALE_FRAME_TIMEOUT_SECONDS) {
            markTargetNotVisible();
        }

        return freshFrameThisLoop;
    }

    private void markTargetNotVisible() {
        if (targetVisible) {
            targetLossTimer.reset();
        }

        angleToTarget = 0.0;
        targetVisible = false;
        detectedTagId = -1;
        tx = 0.0;
        ty = 0.0;
        ta = 0.0;
        horizontalDistance = 0.0;
        settled = false;
        shootReady = false;
    }

    // =========================
    // DERIVATIVE FILTERING
    // =========================
    private void updateDerivative(long currentCaptureTimeMs) {
        double measurement = angleToTarget;

        if (!derivativeInitialized) {
            resetDerivativeState(currentCaptureTimeMs);
            return;
        }

        double dt = (currentCaptureTimeMs - lastCaptureTimeMs) / 1000.0;
        lastVisionDt = dt;

        if (dt < MIN_VALID_VISION_DT || dt > MAX_VALID_VISION_DT) {
            previousMeasurement = measurement;
            filteredRate = 0.0;
            lastRawRate = 0.0;
            return;
        }

        double rawRate = (measurement - previousMeasurement) / dt;
        lastRawRate = rawRate;

        if (Math.abs(rawRate) > MAX_RAW_RATE_DEG_PER_SEC) {
            previousMeasurement = measurement;
            return;
        }

        double alpha = dt / (DERIVATIVE_FILTER_TC + dt);
        filteredRate += alpha * (rawRate - filteredRate);
        filteredRate = clamp(filteredRate, -MAX_FILTERED_RATE_DEG_PER_SEC, MAX_FILTERED_RATE_DEG_PER_SEC);

        previousMeasurement = measurement;
    }

    private void resetDerivativeState(long currentCaptureTimeMs) {
        derivativeInitialized = true;
        previousMeasurement = angleToTarget;
        filteredRate = 0.0;
        lastRawRate = 0.0;
        lastVisionDt = 0.0;
        lastCaptureTimeMs = currentCaptureTimeMs;
    }

    // =========================
    // PD CALCULATION
    // =========================
    private void calculatePD() {
        double controlDt = controlLoopTimer.seconds();
        controlLoopTimer.reset();
        controlDt = clamp(controlDt, 0.001, MAX_CONTROL_DT);
        lastControlDt = controlDt;

        if (!targetVisible) {
            handleTargetLoss(controlDt);
            return;
        }

        double frameAge = freshFrameTimer.seconds();

        double dFreshness = 1.0 - inverseLerp(frameAge, D_FADE_START_SECONDS, D_FADE_END_SECONDS);
        dFreshness = clamp(dFreshness, 0.0, 1.0);

        double error = desiredTx - angleToTarget;
        double absError = Math.abs(error);
        lastError = error;

        double effectiveRate = derivativeInitialized ? filteredRate * dFreshness : 0.0;
        boolean derivativeUsable = derivativeInitialized && dFreshness > 0.0;

        updateSettledState(error, effectiveRate, derivativeUsable);
        updateShootReadyState(error, effectiveRate, derivativeUsable);

        double p = 0.0;
        double d = 0.0;
        double feedForward = 0.0;
        double rawPower;

        double kp = activeKpTurn;
        double kd = activeKdTurn;

        if (absError > activeErrorDeadbandDeg) {
            p = kp * error;
        }

        if (derivativeUsable) {
            d = -kd * effectiveRate;
            d = clamp(d, -MAX_D_TERM_POWER, MAX_D_TERM_POWER);
        }

        if (settled && absError <= activeSettleEnterDeadbandDeg) {
            rawPower = 0.0;
            p = 0.0;
            d = 0.0;
            feedForward = 0.0;
        } else {
            double pdOutput = p + d;

            if (absError > activeErrorDeadbandDeg) {
                feedForward = Math.signum(error) * activeKsTurn;
            } else {
                feedForward = 0.0;
            }

            rawPower = pdOutput + feedForward;

            if (absError <= activeErrorDeadbandDeg) {
                rawPower = 0.0;
            }

            rawPower = clamp(rawPower, -MAX_TURN_POWER, MAX_TURN_POWER);
        }

        lastRawPower = rawPower;
        lastPTerm = p;
        lastDTerm = d;
        lastFeedForward = feedForward;

        applySlewRate(rawPower, controlDt);
    }

    private void updateSettledState(double error, double rate, boolean derivativeFresh) {
        if (!settled) {
            if (derivativeFresh) {
                settled = Math.abs(error) <= activeSettleEnterDeadbandDeg
                        && Math.abs(rate) <= activeSettleEnterRateDps;
            } else {
                settled = Math.abs(error) <= activeSettleEnterDeadbandDeg;
            }
        } else {
            if (derivativeFresh) {
                settled = Math.abs(error) <= activeSettleExitDeadbandDeg
                        && Math.abs(rate) <= activeSettleExitRateDps;
            } else {
                settled = Math.abs(error) <= activeSettleExitDeadbandDeg;
            }
        }
    }

    private void updateShootReadyState(double error, double rate, boolean derivativeFresh) {
        if (!shootReady) {
            if (derivativeFresh) {
                shootReady = Math.abs(error) <= activeShootReadyEnterDeadbandDeg
                        && Math.abs(rate) <= activeShootReadyEnterRateDps;
            } else {
                shootReady = Math.abs(error) <= activeShootReadyEnterDeadbandDeg;
            }
        } else {
            if (derivativeFresh) {
                shootReady = Math.abs(error) <= activeShootReadyExitDeadbandDeg
                        && Math.abs(rate) <= activeShootReadyExitRateDps;
            } else {
                shootReady = Math.abs(error) <= activeShootReadyExitDeadbandDeg;
            }
        }
    }

    // =========================
    // TARGET LOSS & SLEW RATE
    // =========================
    private void handleTargetLoss(double controlDt) {
        double lostFor = targetLossTimer.seconds();
        double targetRawPower = 0.0;

        if (hadTargetPreviously && lostFor <= TARGET_LOST_HOLD_SECONDS) {
            double fade = 1.0 - inverseLerp(lostFor, 0.0, TARGET_LOST_HOLD_SECONDS);
            double held = lastKnownTurnPower * TARGET_LOSS_HOLD_POWER_SCALE * fade;
            targetRawPower = clamp(held, -TARGET_LOSS_MAX_HOLD_POWER, TARGET_LOSS_MAX_HOLD_POWER);
        }

        applySlewRate(targetRawPower, controlDt);

        if (lostFor >= TARGET_LOST_RESET_SECONDS && Math.abs(currentTurnPower) < 0.01) {
            resetPD();
        }

        lastRawPower = targetRawPower;
        lastPTerm = 0.0;
        lastDTerm = 0.0;
        lastFeedForward = 0.0;
        settled = false;
        shootReady = false;
    }

    private void applySlewRate(double targetPower, double dt) {
        double maxStep;

        if (Math.signum(targetPower) != Math.signum(currentTurnPower)
                && targetPower != 0.0
                && currentTurnPower != 0.0) {
            maxStep = MAX_POWER_REVERSE_PER_SEC * dt;
        } else if (Math.abs(targetPower) < Math.abs(currentTurnPower)) {
            maxStep = MAX_POWER_DECEL_PER_SEC * dt;
        } else {
            maxStep = MAX_POWER_ACCEL_PER_SEC * dt;
        }

        double delta = targetPower - currentTurnPower;
        currentTurnPower += clamp(delta, -maxStep, maxStep);
        currentTurnPower = clamp(currentTurnPower, -MAX_TURN_POWER, MAX_TURN_POWER);
        lastKnownTurnPower = currentTurnPower;
    }

    // =========================
    // DISTANCE / ANGLE
    // =========================
    private void calculateDistanceAndAngle() {
        horizontalDistance = calculateDistance(ty);
        angleToTarget = tx;
    }

    private double calculateDistance(double tyAngle) {
        double actualVerticalAngle = CAMERA_TILT_DEGREES + tyAngle;
        double heightDifference = APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        if (Math.abs(actualVerticalAngle) <= 0.1) {
            return Double.POSITIVE_INFINITY;
        }

        double distance = heightDifference / Math.tan(Math.toRadians(actualVerticalAngle));
        return Math.max(0.0, distance);
    }

    // =========================
    // RESET & HELPERS
    // =========================
    private void resetPD() {
        derivativeInitialized = false;
        filteredRate = 0.0;
        previousMeasurement = angleToTarget;
        lastCaptureTimeMs = 0;
        currentTurnPower = 0.0;
        lastKnownTurnPower = 0.0;
        lastPTerm = 0.0;
        lastDTerm = 0.0;
        lastFeedForward = 0.0;
        lastError = 0.0;
        lastVisionDt = 0.0;
        lastControlDt = 0.0;
        lastRawRate = 0.0;
        lastRawPower = 0.0;
        hadTargetPreviously = false;
        previousDetectedTagId = -1;
        settled = false;
        shootReady = false;
        freshFrameThisLoop = false;
        markTargetNotVisible();
    }

    public void sendTelemetry() {
        telemetry.addData("LL Visible", targetVisible);
        telemetry.addData("LL Fresh", freshFrameThisLoop);
        telemetry.addData("LL Tag", detectedTagId);
        telemetry.addData("LL tx", tx);
        telemetry.addData("LL Error", lastError);
        telemetry.addData("LL Rate", filteredRate);
        telemetry.addData("LL P", lastPTerm);
        telemetry.addData("LL D", lastDTerm);
        telemetry.addData("LL FF", lastFeedForward);
        telemetry.addData("LL RawPower", lastRawPower);
        telemetry.addData("LL TurnPower", currentTurnPower);
        telemetry.addData("LL Settled", settled);
        telemetry.addData("LL ShootReady", shootReady);
        telemetry.addData("LL VisionDt", lastVisionDt);
        telemetry.addData("LL CtrlDt", lastControlDt);
        telemetry.addData("LL FrameAge", freshFrameTimer.seconds());
        telemetry.addData(
                "LL DFreshness",
                clamp(1.0 - inverseLerp(freshFrameTimer.seconds(), D_FADE_START_SECONDS, D_FADE_END_SECONDS), 0.0, 1.0)
        );
        telemetry.addData("LL Distance", horizontalDistance);
        telemetry.addData("LL RobotY", robotY);
        telemetry.addData("LL AimProfile", useFastAimProfile ? "FAST" : "PRECISE");
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double inverseLerp(double x, double a, double b) {
        if (a == b) return 0.0;
        return clamp((x - a) / (b - a), 0.0, 1.0);
    }

    // =========================
    // PUBLIC GETTERS
    // =========================
    public double getTurnPower() { return currentTurnPower; }
    public boolean isTargetVisible() { return targetVisible; }
    public int getDetectedTagId() { return detectedTagId; }
    public double getTx() { return tx; }
    public double getFilteredRate() { return filteredRate; }
    public double getLastError() { return lastError; }
    public boolean isFreshFrameThisLoop() { return freshFrameThisLoop; }
    public double getHorizontalDistance() { return horizontalDistance; }
    public boolean isSettled() { return settled; }
    public boolean isShootReady() { return shootReady; }
    public String getAimProfileName() { return useFastAimProfile ? "FAST" : "PRECISE"; }

}