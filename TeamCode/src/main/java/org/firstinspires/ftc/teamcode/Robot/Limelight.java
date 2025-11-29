package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private final Telemetry telemetry;

    // --- CAMERA CONFIGURATION ---
    private static final double CAMERA_TILT_DEGREES = 0.0; // Camera tilted up 0 degrees
    private static final double CAMERA_HEIGHT_INCHES = 10.5; // Height of camera from ground (adjust this)
    private static final double APRILTAG_HEIGHT_INCHES = 38.5; // Height of AprilTag from ground (adjust this)

    // --- ALIGNMENT TUNING CONSTANTS ---
    // These are proportional constants (Kp) for simple P-control
    private static final double Kp_TURN = 0.02; // Tune this: Power per degree of angleToTarget
    private static final double Kp_DRIVE = 0.05; // Tune this: Power per inch of distance error
    private static final double DRIVE_MIN_POWER = 0.10; // Minimum power to overcome friction/start moving

    // AprilTag detection results
    private LLResult result;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private double tx = 0.0; // Horizontal offset from crosshair to target
    private double ty = 0.0; // Vertical offset from crosshair to target (in camera frame)
    private double ta = 0.0; // Target area (0-100% of image)

    // Calculated values for robot control
    private double horizontalDistance = 0.0; // Ground distance to target
    private double angleToTarget = 0.0; // How many degrees robot needs to turn

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        limelight.pipelineSwitch(4); // Switch to AprilTag pipeline
        limelight.start(); // Start the Limelight
    }

    /**
     * Update AprilTag detection data from Limelight
     * Call this method periodically in your loop
     */
    public void update() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            targetVisible = true;

            // Get first detected AprilTag
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducialResults.get(0);

                detectedTagId = (int) tag.getFiducialId();
                tx = tag.getTargetXDegrees();
                ty = tag.getTargetYDegrees();
                ta = tag.getTargetArea();

                // Calculate real-world distance and angle
                calculateDistanceAndAngle();
            } else {
                // If result is valid but no fiducial results (maybe just another pipeline item)
                resetTargetData();
            }
        } else {
            // If result is null or invalid
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
    }

    /**
     * Calculate horizontal distance to AprilTag and angle offset
     * Uses geometry based on camera tilt and vertical angle
     *
     */
    private void calculateDistanceAndAngle() {
        // Calculate the actual angle to target in world space
        // ty is the vertical offset in camera frame
        // We need to account for the CAMERA_TILT_DEGREES (e.g., 45-degree tilt)
        double actualVerticalAngle = CAMERA_TILT_DEGREES - ty;

        // Height difference between camera and AprilTag
        double heightDifference = APRILTAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        // Calculate horizontal distance using trigonometry
        // tan(angle) = opposite / adjacent
        // adjacent = opposite / tan(angle)
        if (Math.abs(actualVerticalAngle) > 0.1) { // Check for near-zero angle to avoid division by zero/large distance
            horizontalDistance = heightDifference / Math.tan(Math.toRadians(actualVerticalAngle));
        } else {
            horizontalDistance = Double.MAX_VALUE; // Far away or perfectly level
        }

        // The horizontal offset (tx) directly represents how many degrees
        // the robot needs to turn to center the AprilTag
        angleToTarget = tx;
    }

    // --- ALIGNMENT METHODS ---

    /**
     * Calculates the turn power needed to center the target using P-control.
     * @return Turn power (-1.0 to 1.0). Positive power turns Right.
     */
    public double getTurnPower() {
        if (!targetVisible) {
            return 0.0;
        }

        // Simple P-control: Power = Kp * Error
        double turnPower = angleToTarget * Kp_TURN;

        // Clamp the power to be between -1.0 and 1.0
        return Math.max(-1.0, Math.min(1.0, turnPower));
    }

    public double getDrivePower(double targetDistance) {
        if (!targetVisible) {
            return 0.0;
        }

        // Error = Target - Actual (or Actual - Target, depending on desired direction)
        // We want positive power when we are too far (Actual > Target)
        // Error = CurrentDistance - TargetDistance
        double distanceError = horizontalDistance - targetDistance;

        // Simple P-control: Power = Kp * Error
        double drivePower = distanceError * Kp_DRIVE;

        // Apply a minimum power to overcome stiction/friction if we're moving
        if (Math.abs(drivePower) > 0.01) { // Only apply if a command is given
            if (drivePower > 0) { // Driving forward
                drivePower = Math.max(DRIVE_MIN_POWER, drivePower);
            } else { // Driving backward
                drivePower = Math.min(-DRIVE_MIN_POWER, drivePower);
            }
        }


        // Clamp the power to be between -1.0 and 1.0
        return Math.max(-1.0, Math.min(1.0, drivePower));
    }

    // --- GETTERS (Original methods remain) ---

    public boolean isTargetVisible() {
        return targetVisible;
    }
    // ... (All other original getters and methods remain the same) ...
    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Get horizontal offset angle (how many degrees robot needs to turn)
     * Positive = target is to the right, negative = target is to the left
     */
    public double getAngleToTarget() {
        return angleToTarget;
    }

    /**
     * Get horizontal distance to AprilTag on the ground plane (in inches)
     * This is the actual distance the robot would need to drive forward
     */
    public double getHorizontalDistance() {
        return horizontalDistance;
    }

    /**
     * Get raw tx value (horizontal offset from crosshair in degrees)
     */
    public double getTx() {
        return tx;
    }

    /**
     * Get raw ty value (vertical offset from crosshair in degrees)
     */
    public double getTy() {
        return ty;
    }

    /**
     * Get target area as percentage of image (0-100)
     */
    public double getTa() {
        return ta;
    }

    /**
     * Check if a specific AprilTag ID is visible
     */
    public boolean isTagVisible(int targetId) {
        return targetVisible && detectedTagId == targetId;
    }

    /**
     * Check if robot is centered on target (within tolerance)
     * @param toleranceDegrees Acceptable angle offset (e.g., 2.0 degrees)
     */
    public boolean isCenteredOnTarget(double toleranceDegrees) {
        return targetVisible && Math.abs(angleToTarget) <= toleranceDegrees;
    }

    /**
     * Check if robot is at target distance (within tolerance)
     * @param targetDistance Desired distance in inches
     * @param toleranceInches Acceptable distance error (e.g., 2.0 inches)
     */
    public boolean isAtTargetDistance(double targetDistance, double toleranceInches) {
        return targetVisible && Math.abs(horizontalDistance - targetDistance) <= toleranceInches;
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public List<LLResultTypes.FiducialResult> getAllDetectedTags() {
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    public void displayTelemetry() {
        telemetry.addData("Limelight", "Status: %s", limelight.isConnected() ? "Connected" : "Disconnected");
        telemetry.addData("Target Visible", targetVisible);

        if (targetVisible) {
            telemetry.addData("AprilTag ID", detectedTagId);
            telemetry.addData("--- Robot Control Data ---", "");
            telemetry.addData("Distance to Target", "%.2f inches", horizontalDistance);
            telemetry.addData("Angle to Turn", "%.2f degrees %s",
                    Math.abs(angleToTarget),
                    angleToTarget > 0 ? "(Right)" : angleToTarget < 0 ? "(Left)" : "(Centered)");
            telemetry.addData("--- Raw Camera Data ---", "");
            telemetry.addData("Horizontal Offset (tx)", "%.2f degrees", tx);
            telemetry.addData("Vertical Offset (ty)", "%.2f degrees", ty);
            telemetry.addData("Target Area", "%.2f%%", ta);
        }
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
