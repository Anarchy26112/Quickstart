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

    // Settled hysteresis
    private static final double SETTLE_ENTER_DEADBAND_DEGREES = 1.0;
    private static final double SETTLE_EXIT_DEADBAND_DEGREES = 1.2;
    private static final double SETTLE_ENTER_RATE_DEG_PER_SEC = 2.5;
    private static final double SETTLE_EXIT_RATE_DEG_PER_SEC = 4.5;
    private boolean settled = false;

    // Shoot-ready hysteresis
    private static final double SHOOT_READY_ENTER_DEADBAND_DEGREES = 3.5;
    private static final double SHOOT_READY_EXIT_DEADBAND_DEGREES = 4.2;
    private static final double SHOOT_READY_ENTER_RATE_DEG_PER_SEC = 10.0;
    private static final double SHOOT_READY_EXIT_RATE_DEG_PER_SEC = 15.0;
    private boolean shootReady = false;

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

    // Deadbands
    private static final double ERROR_DEADBAND_DEGREES = 0.32;

    // Target loss hold
    private static final double TARGET_LOSS_HOLD_POWER_SCALE = 0.22;
    private static final double TARGET_LOSS_MAX_HOLD_POWER = 0.08;

    // Gain scheduling zones
    private static final double NEAR_ZONE_DEGREES = 4.0;
    private static final double FAR_ZONE_DEGREES = 15.0;

    // Large setpoint step threshold
    private static final double LARGE_SETPOINT_STEP_DEG = 2.0;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        this.limelight.pipelineSwitch(4);
        this.limelight.start();

        targetLossTimer.reset();
        controlLoopTimer.reset();
        freshFrameTimer.reset();
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

        // Early-out for repeated stale frame:
        // still preserve timeout behavior but skip rescanning tags and redoing math.
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

            // Only do measurement math on a fresh frame
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

        // Treat repeated stale frames as lost target
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

        // Fade derivative influence as the frame gets stale
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

        // Live gain scheduling
        double kpNear = 0.0085;
        double kpFar  = 0.0115;

        double kdNear = 0.0009;
        double kdFar  = 0.0006;

        double zoneT = inverseLerp(absError, NEAR_ZONE_DEGREES, FAR_ZONE_DEGREES);
        zoneT = clamp(zoneT, 0.0, 1.0);

        double kp = lerp(kpNear, kpFar, zoneT);
        double kd = lerp(kdNear, kdFar, zoneT);

        // P term
        if (absError > ERROR_DEADBAND_DEGREES) {
            p = kp * error;
        }

        // D term
        if (derivativeUsable) {
            d = -kd * effectiveRate;
            d = clamp(d, -MAX_D_TERM_POWER, MAX_D_TERM_POWER);
        }

        // Settled hold
        if (settled && absError <= SETTLE_ENTER_DEADBAND_DEGREES) {
            rawPower = 0.0;
            p = 0.0;
            d = 0.0;
            feedForward = 0.0;
        } else {
            double pdOutput = p + d;

            // Add kS if we are outside the deadband
            if (absError > ERROR_DEADBAND_DEGREES) {
                feedForward = Math.signum(error) * kS_VOLTAGE_COMP;
            } else {
                feedForward = 0.0;
            }

            rawPower = pdOutput + feedForward;

            // Hard zero when very close to prevent micro-jitter
            if (absError <= ERROR_DEADBAND_DEGREES) {
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
                settled = Math.abs(error) <= SETTLE_ENTER_DEADBAND_DEGREES
                        && Math.abs(rate) <= SETTLE_ENTER_RATE_DEG_PER_SEC;
            } else {
                settled = Math.abs(error) <= SETTLE_ENTER_DEADBAND_DEGREES;
            }
        } else {
            if (derivativeFresh) {
                settled = Math.abs(error) <= SETTLE_EXIT_DEADBAND_DEGREES
                        && Math.abs(rate) <= SETTLE_EXIT_RATE_DEG_PER_SEC;
            } else {
                settled = Math.abs(error) <= SETTLE_EXIT_DEADBAND_DEGREES;
            }
        }
    }

    private void updateShootReadyState(double error, double rate, boolean derivativeFresh) {
        if (!shootReady) {
            if (derivativeFresh) {
                shootReady = Math.abs(error) <= SHOOT_READY_ENTER_DEADBAND_DEGREES
                        && Math.abs(rate) <= SHOOT_READY_ENTER_RATE_DEG_PER_SEC;
            } else {
                shootReady = Math.abs(error) <= SHOOT_READY_ENTER_DEADBAND_DEGREES;
            }
        } else {
            if (derivativeFresh) {
                shootReady = Math.abs(error) <= SHOOT_READY_EXIT_DEADBAND_DEGREES
                        && Math.abs(rate) <= SHOOT_READY_EXIT_RATE_DEG_PER_SEC;
            } else {
                shootReady = Math.abs(error) <= SHOOT_READY_EXIT_DEADBAND_DEGREES;
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
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
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
}