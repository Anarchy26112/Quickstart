package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private final Telemetry telemetry;

    // --- CAMERA CONFIGURATION ---
    private static final double CAMERA_TILT_DEGREES = 0.0;
    private static final double CAMERA_HEIGHT_INCHES = 10.25;
    private static final double APRILTAG_HEIGHT_INCHES = 29.5;

    // --- STATE VARIABLES ---
    private LLResult result;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;

    // Calculated State
    public double horizontalDistance = 0.0;
    private double angleToTarget = 0.0;
    private double lastResultTimestampSec = 0.0;

    // --- PID CONTROL STATE ---
    private double currentTurnPower = 0.0; // The calculated output
    private double desiredTx = 0.0;        // The setpoint (0.0 = center)

    private double integral = 0.0;
    private double previousError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean firstUpdate = true;

    // --- DERIVATIVE FILTERING ---
    private double filteredRate = 0.0;
    private double previousMeasurement = 0.0; // For Derivative-on-Measurement

    private static final double DERIVATIVE_ALPHA = 0.6;

    // --- PID SAFETY LIMITS ---
    private static final double MAX_INTEGRAL = 200.0;
    private static final double MAX_P_OUTPUT = 0.5;
    private static final double MAX_I_OUTPUT = 0.3;
    private static final double MAX_D_OUTPUT = 0.4;
    private static final double DEADBAND_DEGREES = 1.3;

    // --- TARGET LOSS HANDLING ---
    private static final double TARGET_LOST_HOLD_SECONDS = 0.10;
    private static final double TARGET_LOST_RESET_SECONDS = 0.25;
    private final ElapsedTime targetLossTimer = new ElapsedTime();
    private boolean hadTargetPreviously = false;
    private double lastKnownTurnPower = 0.0; // To hold value during signal loss

    // --- DEBUG STORAGE ---
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastError = 0.0;

    // --- TARGET FILTERING ---
    private final List<Integer> allowedTagIds = new ArrayList<>();

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        limelight.pipelineSwitch(4);
        limelight.start();

        pidTimer.reset();
        targetLossTimer.reset();
    }

    // --- CONFIGURATION ---

    public void setTargetBlue() { setAllowedTags(20); }
    public void setTargetRed() { setAllowedTags(24); }
    public void setTargetMotif() { setAllowedTags(21, 22, 23); }
    public void trackAnyTag() { allowedTagIds.clear(); }

    private void setAllowedTags(Integer... tags) {
        allowedTagIds.clear();
        allowedTagIds.addAll(Arrays.asList(tags));
    }

    /**
     * Set the desired alignment angle.
     * 0.0 = center of image.
     * Positive = aim right, Negative = aim left.
     */
    public void setTargetAngle(double angleDegrees) {
        this.desiredTx = angleDegrees;
    }

    // --- MAIN UPDATE LOOP ---

    /**
     * Call this ONCE per loop. It reads the camera and calculates the PID.
     */
    public void update() {
        // 1. READ CAMERA
        result = limelight.getLatestResult();
        processVisionResult(result);

        // 2. CALCULATE PID
        calculatePID();
    }

    private void processVisionResult(LLResult result) {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            LLResultTypes.FiducialResult matchingTag = null;

            for (LLResultTypes.FiducialResult tag : fiducialResults) {
                int id = (int) tag.getFiducialId();
                if (allowedTagIds.isEmpty() || allowedTagIds.contains(id)) {
                    matchingTag = tag;
                    break;
                }
            }

            if (matchingTag != null) {
                targetVisible = true;
                detectedTagId = (int) matchingTag.getFiducialId();
                tx = matchingTag.getTargetXDegrees();
                ty = matchingTag.getTargetYDegrees();
                ta = matchingTag.getTargetArea();

                calculateDistanceAndAngle();

                targetLossTimer.reset();
                hadTargetPreviously = true;
                return;
            }
        }

        targetVisible = false;
        detectedTagId = -1;
    }

    private void calculatePID() {
        // --- 1. HANDLE TARGET LOSS ---
        if (!targetVisible) {
            double lostFor = targetLossTimer.seconds();
            if (!hadTargetPreviously || lostFor >= TARGET_LOST_RESET_SECONDS) {
                // Completely lost
                resetPID();
                currentTurnPower = 0.0;
                lastKnownTurnPower = 0.0;
            } else if (lostFor <= TARGET_LOST_HOLD_SECONDS) {
                // Hold last power
                currentTurnPower = lastKnownTurnPower;
                if (Math.abs(lastError) >= 5.0) currentTurnPower *= 0.5; // Safety dampening
            } else {
                // Decay
                double t = (lostFor - TARGET_LOST_HOLD_SECONDS) /
                        (TARGET_LOST_RESET_SECONDS - TARGET_LOST_HOLD_SECONDS);
                currentTurnPower = lastKnownTurnPower * (1.0 - clamp(t, 0.0, 1.0));
            }
            return;
        }

        // --- 2. TIME DELTA ---

        double error = desiredTx - angleToTarget;
        lastError = error;

        // --- 3. FIRST LOOP INIT ---
        if (firstUpdate) {
            firstUpdate = false;
            previousError = error;
            previousMeasurement = angleToTarget;
            if (result != null) {
                lastResultTimestampSec = result.getTimestamp() / 1000.0;
            }
            // Start simple to avoid jumps
            currentTurnPower = clamp(error * Kp_TURN, -1.0, 1.0);
            return;
        }

        // --- 4. PID TERMS ---

        // P Term
        double p = error * Kp_TURN;
        p = clamp(p, -MAX_P_OUTPUT, MAX_P_OUTPUT);

        double i = 0;
        i = clamp(i, -MAX_I_OUTPUT, MAX_I_OUTPUT);

        // D Term (Derivative on Measurement)
        double d = 0.0;
        double currentTsSec = (result != null) ? (result.getTimestamp() / 1000.0) : 0.0;

        // Only calculate derivative if we have a NEW camera frame
        if (currentTsSec > 0 && lastResultTimestampSec > 0 && currentTsSec != lastResultTimestampSec) {
            double dtCamera = currentTsSec - lastResultTimestampSec;

            // Avoid divide by zero or massive spikes
            if (dtCamera > 0.005 && dtCamera < 0.5) {
                double rate = (angleToTarget - previousMeasurement) / dtCamera;

                // Low Pass Filter
                filteredRate = (DERIVATIVE_ALPHA * filteredRate) + ((1.0 - DERIVATIVE_ALPHA) * rate);
            }

            // Update state ONLY on new frame
            previousMeasurement = angleToTarget;
            lastResultTimestampSec = currentTsSec;
        }

        d = (-filteredRate) * Kd_TURN;
        d = clamp(d, -MAX_D_OUTPUT, MAX_D_OUTPUT);

        // --- 5. SUM AND OUTPUT ---
        double rawPower = p + i + d;

        // Stiction / Min Power (only if outside deadband)
        if (Math.abs(error) > DEADBAND_DEGREES && Math.abs(rawPower) < MIN_TURN_POWER) {
            rawPower = Math.signum(rawPower) * MIN_TURN_POWER;
        } else if (Math.abs(error) <= DEADBAND_DEGREES) {
            rawPower = 0.0;
        }

        currentTurnPower = clamp(rawPower, -1.0, 1.0);
        lastKnownTurnPower = currentTurnPower;

        // Store for Telemetry
        lastPTerm = p;
        lastITerm = i;
        lastDTerm = d;
    }

    // --- GETTERS & HELPERS ---

    public double getTurnPower() {
        return currentTurnPower;
    }

    public double getTurnPowerOffsetByInches(double inchesOffset) {
        if (!targetVisible || horizontalDistance < 1.0) return 0.0;

        // Calculate the angle required for this offset
        double angleOffsetDeg = Math.toDegrees(Math.atan2(inchesOffset, horizontalDistance));

        // Set it as the target (for telemetry/tracking consistency)
        setTargetAngle(angleOffsetDeg);

        // Return current power (Note: This returns the power calculated in update(),
        // so it might lag one loop behind the new target set here. That is usually fine.)
        return currentTurnPower;
    }

    private double calculateDistance(double tyAngle) {
        double actualVerticalAngle = CAMERA_TILT_DEGREES + tyAngle;
        double heightDifference = APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        if (Math.abs(actualVerticalAngle) > 0.1) {
            return heightDifference / Math.tan(Math.toRadians(actualVerticalAngle));
        } else {
            return Double.MAX_VALUE;
        }
    }

    private void calculateDistanceAndAngle() {
        horizontalDistance = calculateDistance(ty);
        angleToTarget = tx;
    }

    private void resetPID() {
        integral = 0.0;
        previousError = 0.0;
        filteredRate = 0.0;
        firstUpdate = true;
        lastResultTimestampSec = 0;
        currentTurnPower = 0.0;
        pidTimer.reset();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public boolean isTargetVisible() { return targetVisible; }
    public int getDetectedTagId() { return detectedTagId; }
    public double getTx() { return tx; }

    public boolean isCenteredOnTarget(double toleranceDegrees) {
        return targetVisible && Math.abs(lastError) <= toleranceDegrees;
    }

    public void displayTelemetry() {
        telemetry.addData("Limelight", "Status: %s", limelight.isConnected() ? "Connected" : "Disconnected");
        telemetry.addData("Target Found", targetVisible);

        if (targetVisible) {
            telemetry.addData("ID", detectedTagId);
            telemetry.addData("Dist", "%.2f in", horizontalDistance);
            telemetry.addData("TX", "%.2f (Target: %.2f)", tx, desiredTx);
            telemetry.addData("PID Out", "%.3f", currentTurnPower);
            telemetry.addData("Terms", "P:%.3f I:%.3f D:%.3f", lastPTerm, lastITerm, lastDTerm);
        }
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}