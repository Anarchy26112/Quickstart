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
    private static final double CAMERA_HEIGHT_INCHES = 10.5;
    private static final double APRILTAG_HEIGHT_INCHES = 29.5;

    // AprilTag detection results
    private LLResult result;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;

    // Calculated values for robot control
    private double horizontalDistance = 0.0;
    private double angleToTarget = 0.0;

    // --- PID CONTROL VARIABLES ---
    private double integral = 0.0;
    private double previousError = 0.0;
    private double lastReference = 0.0;  // Track setpoint changes
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean firstUpdate = true;

    // --- LOW-PASS FILTER VARIABLES ---
    private static final double FILTER_COEFFICIENT = 0.8;  // 0 < a < 1
    private double previousFilterEstimate = 0.0;
    private double currentFilterEstimate = 0.0;

    // --- PID SAFETY LIMITS ---
    private static final double MAX_INTEGRAL = 200.0;
    private static final double MIN_DT = 0.001;  // Minimum delta time to prevent spikes

    // Per-term output limits
    private static final double MAX_P_OUTPUT = 0.5;
    private static final double MAX_I_OUTPUT = 0.3;
    private static final double MAX_D_OUTPUT = 0.4;

    // --- MINIMUM SPEED THRESHOLD ---
    // Below this threshold, output will be zero to overcome friction
    private static final double MIN_TURN_POWER = 0.08;  // Adjust based on your robot
    private static final double DEADBAND_DEGREES = 0.5;  // Stop completely when within this angle

    // --- PID TERM STORAGE (for telemetry) ---
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastTurnPower = 0.0;
    private double lastRawPower = 0.0;  // Before minimum threshold applied

    // --- TARGET FILTERING ---
    private List<Integer> allowedTagIds = new ArrayList<>();

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        limelight.pipelineSwitch(4);
        limelight.start();

        pidTimer.reset();
    }

    public void setTargetBlue() {
        setAllowedTags(20);
    }

    public void setTargetRed() {
        setAllowedTags(24);
    }

    public void setTargetMotif() {
        setAllowedTags(21, 22, 23);
    }

    /**
     * Clear filters (track any visible tag).
     */
    public void trackAnyTag() {
        allowedTagIds.clear();
    }

    /**
     * Internal helper to set specific IDs
     */
    private void setAllowedTags(Integer... tags) {
        allowedTagIds.clear();
        allowedTagIds.addAll(Arrays.asList(tags));
    }

    /**
     * Update AprilTag detection data from Limelight
     * Filters based on allowedTagIds and uses the first matching tag.
     */
    public void update() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            // Find the first tag that matches our filter
            LLResultTypes.FiducialResult matchingTag = null;

            for (LLResultTypes.FiducialResult tag : fiducialResults) {
                int id = (int) tag.getFiducialId();

                // Check if we are filtering, and if so, does this tag match?
                if (allowedTagIds.isEmpty() || allowedTagIds.contains(id)) {
                    matchingTag = tag;
                    break; // Only one matching tag will be on field
                }
            }

            if (matchingTag != null) {
                targetVisible = true;
                detectedTagId = (int) matchingTag.getFiducialId();
                tx = matchingTag.getTargetXDegrees();
                ty = matchingTag.getTargetYDegrees();
                ta = matchingTag.getTargetArea();

                calculateDistanceAndAngle();
            } else {
                // Valid result frame, but no tags matched our specific filter
                resetTargetData();
            }

        } else {
            resetTargetData();
        }
    }

    private void resetTargetData() {
        targetVisible = false;
        detectedTagId = -1;
        tx = 0.0;
        ty = 0.0;
        ta = 0.0;
        horizontalDistance = 0.0;
        angleToTarget = 0.0;

        // Reset PID variables when target is lost
        resetPID();
    }

    /**
     * Helper method to calculate distance from ty angle
     */
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

    // --- PID RESET METHOD
    private void resetPID() {
        integral = 0.0;
        previousError = 0.0;
        lastReference = 0.0;
        previousFilterEstimate = 0.0;
        currentFilterEstimate = 0.0;
        firstUpdate = true;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
        lastTurnPower = 0.0;
        lastRawPower = 0.0;
    }

    // --- ALIGNMENT METHODS ---

    public double getTurnPower() {
        if (!targetVisible) {
            resetPID();
            return 0.0;
        }

        if (pidTimer.seconds() < 0.005) {
            return lastTurnPower;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Calculate error (reference is 0, we want to center on target)
        double reference = 0.0;
        double error = reference - angleToTarget;

        // Check if we're within deadband - if so, stop completely
        if (Math.abs(error) < DEADBAND_DEGREES) {
            resetPID();
            lastTurnPower = 0.0;
            lastRawPower = 0.0;
            return 0.0;
        }

        // Skip PID calculation on first update to avoid derivative spike
        if (firstUpdate) {
            firstUpdate = false;
            previousError = error;
            lastReference = reference;
            double simplePower = error * Kp_TURN;
            lastPTerm = simplePower;
            lastITerm = 0.0;
            lastDTerm = 0.0;
            lastRawPower = simplePower;

            // Apply minimum threshold
            if (Math.abs(simplePower) < MIN_TURN_POWER) {
                simplePower = Math.signum(simplePower) * MIN_TURN_POWER;
            }

            lastTurnPower = Math.max(-1.0, Math.min(1.0, simplePower));
            return lastTurnPower;
        }

        // ===== PROPORTIONAL TERM =====
        double p = error * Kp_TURN;
        p = Math.max(-MAX_P_OUTPUT, Math.min(MAX_P_OUTPUT, p));

        // ===== INTEGRAL TERM =====
        integral += error * dt;

        // Anti-windup: clamp integral to prevent excessive buildup
        if (integral > MAX_INTEGRAL) {
            integral = MAX_INTEGRAL;
        }
        if (integral < -MAX_INTEGRAL) {
            integral = -MAX_INTEGRAL;
        }

        // Reset integral upon setpoint changes
        if (reference != lastReference) {
            integral = 0.0;
        }

        double i = integral * Ki_TURN;
        i = Math.max(-MAX_I_OUTPUT, Math.min(MAX_I_OUTPUT, i));

        // ===== DERIVATIVE TERM =====
        double errorChange = error - previousError;

        // Filter out high frequency noise to increase derivative performance
        currentFilterEstimate = (FILTER_COEFFICIENT * previousFilterEstimate) +
                (1.0 - FILTER_COEFFICIENT) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // Rate of change of the error
        double derivative = 0.0;
        if (dt > MIN_DT) {  // Ignore very small dt to prevent spikes
            derivative = currentFilterEstimate / dt;
        }

        double d = derivative * Kd_TURN;
        d = Math.max(-MAX_D_OUTPUT, Math.min(MAX_D_OUTPUT, d));

        // Store for telemetry
        lastPTerm = p;
        lastITerm = i;
        lastDTerm = d;

        // Update previous values
        previousError = error;
        lastReference = reference;

        // Combine PID terms
        double turnPower = p + i + d;
        lastRawPower = turnPower;

        // Apply Minimum Feedforward (Stiction recovery)
        if (Math.abs(turnPower) < MIN_TURN_POWER) {
            turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
        }

        // Clamp final output to [-1, 1]
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
        lastTurnPower = turnPower;

        return turnPower;
    }

    // --- GETTERS ---

    public boolean isTargetVisible() { return targetVisible; }
    public int getDetectedTagId() { return detectedTagId; }
    public double getAngleToTarget() { return angleToTarget; }
    public double getHorizontalDistance() { return horizontalDistance; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double getIntegral() { return integral; }
    public double getPreviousError() { return previousError; }

    public boolean isCenteredOnTarget(double toleranceDegrees) {
        return targetVisible && Math.abs(angleToTarget) <= toleranceDegrees;
    }

    public boolean isAtTargetDistance(double targetDistance, double toleranceInches) {
        return targetVisible && Math.abs(horizontalDistance - targetDistance) <= toleranceInches;
    }

    public void displayTelemetry() {
        telemetry.addData("Limelight", "Status: %s", limelight.isConnected() ? "Connected" : "Disconnected");

        // Show what we are looking for
        String lookingFor = allowedTagIds.isEmpty() ? "ANY" : allowedTagIds.toString();
        telemetry.addData("Target Filter", lookingFor);
        telemetry.addData("Target Visible", targetVisible);

        if (targetVisible) {
            telemetry.addData("AprilTag ID", detectedTagId);
            telemetry.addData("Distance", "%.2f in", horizontalDistance);
            telemetry.addData("Angle", "%.2f deg", angleToTarget);
            telemetry.addData("--- PID Debug ---", "");
            telemetry.addData("Raw Power", "%.3f", lastRawPower);
            telemetry.addData("Turn Power (w/ threshold)", "%.3f", lastTurnPower);
            telemetry.addData("P term", "%.3f", lastPTerm);
            telemetry.addData("I term", "%.3f (int: %.1f)", lastITerm, integral);
            telemetry.addData("D term", "%.3f (filter: %.3f)", lastDTerm, currentFilterEstimate);
        }
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}