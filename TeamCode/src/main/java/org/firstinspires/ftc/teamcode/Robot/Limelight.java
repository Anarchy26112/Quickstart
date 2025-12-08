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
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean firstUpdate = true;

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
        double actualVerticalAngle = CAMERA_TILT_DEGREES + tyAngle;  // Fixed: + tyAngle for correct sign (ty positive = above)
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
        firstUpdate = true;
    }

    // --- ALIGNMENT METHODS ---

    public double getTurnPower() {
        if (!targetVisible) {
            resetPID();
            return 0.0;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Skip PID calculation on first update to avoid derivative spike
        if (firstUpdate) {
            firstUpdate = false;
            previousError = -angleToTarget;
            return -angleToTarget * Kp_TURN;
        }

        // PID calculations
        double error = -angleToTarget;

        // Proportional term
        double p = error * Kp_TURN;

        // Integral term (accumulate error over time)
        integral += error * dt;
        // Anti-windup: clamp integral to prevent excessive buildup
        double maxIntegral = 1.0 / Math.abs(Ki_TURN);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double i = integral * Ki_TURN;

        // Derivative term (rate of change of error)
        double derivative = 0.0;
        if (dt > 0.0) {
            derivative = (error - previousError) / dt;
        }
        double d = derivative * Kd_TURN;

        previousError = error;

        // Combine PID terms
        double turnPower = p + i + d;

        // Clamp output to [-1, 1]
        return Math.max(-1.0, Math.min(1.0, turnPower));
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
            telemetry.addData("PID Integral", "%.4f", integral);
        }
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}