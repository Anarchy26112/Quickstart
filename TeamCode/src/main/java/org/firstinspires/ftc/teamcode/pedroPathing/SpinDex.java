package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SpinDex {
    private Servo spin_dex;
    private ColorRangeSensor ballColorSensor;
    private TelemetryManager telemetryM;

    // Hardware configuration
    private static final String SERVO_NAME = "spinDex";
    private static final String COLOR_SENSOR_NAME = "ballColorSensor";

    // State tracking
    public String[] slots = {"empty", "empty", "empty"}; // 6 slots
    private int currentPosition = 1; // Start at home
    private int currentTurn = 0;

    // Servo position constants
    private static final int POSITIONS_PER_TURN = 6;
    private static final int MAX_POSITION = 23;
    private static final int MIN_POSITION = 6;
    private static final double SERVO_SCALE = 30.0;
    private static final int OPTIMAL_DISTANCE = 15;

    // Color detection thresholds
    private static final int GREEN_THRESHOLD = 100;
    private static final int PURPLE_RED_THRESHOLD = 50;
    private static final int PURPLE_BLUE_THRESHOLD = 50;
    private static final double PURPLE_RATIO_THRESHOLD = 1.5;

    public SpinDex(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetryM = telemetry;

        // Initialize servo
        spin_dex = hardwareMap.get(Servo.class, SERVO_NAME);

        // Initialize color sensor
        ballColorSensor = hardwareMap.get(ColorRangeSensor.class, COLOR_SENSOR_NAME);
    }

    /**
     * Move to a target position (1-6)
     */
    public void moveToPosition(int targetPosition) {
        if (targetPosition < 1 || targetPosition > POSITIONS_PER_TURN) {
            telemetryM.debug("Error: Invalid position " + targetPosition + " (valid range: 1-" + POSITIONS_PER_TURN + ")");
            return;
        }

        if (spin_dex == null) {
            telemetryM.debug("Error: Cannot move - servo not initialized");
            return;
        }

        if (currentPosition < targetPosition) {
            sortBallsForward(targetPosition);
        } else if (currentPosition > targetPosition) {
            sortBallsBackward(targetPosition);
        }
        // If equal, do nothing
    }

    /**
     * Rotate forward to target position
     */
    private void sortBallsForward(int targetPosition) {
        int currentValue = currentTurn * POSITIONS_PER_TURN + currentPosition;
        int target1 = (currentTurn - 1) * POSITIONS_PER_TURN + targetPosition;
        int target2 = currentTurn * POSITIONS_PER_TURN + targetPosition;

        int distance1 = Math.abs(target1 - currentValue);
        int distance2 = Math.abs(target2 - currentValue);

        // Determine which target to use based on constraints and distance
        if (target1 < MIN_POSITION) {
            // Can only go to target2
            setServoPosition(target2, targetPosition, 0);
        } else if (target2 > MAX_POSITION) {
            // Can only go to target1
            setServoPosition(target1, targetPosition, -1);
        } else {
            // Both targets valid, choose based on distance
            chooseOptimalTarget(target1, target2, distance1, distance2, targetPosition, -1, 0);
        }
    }

    /**
     * Rotate backward to target position
     */
    private void sortBallsBackward(int targetPosition) {
        int currentValue = currentTurn * POSITIONS_PER_TURN + currentPosition;
        int target1 = currentTurn * POSITIONS_PER_TURN + targetPosition;
        int target2 = (currentTurn + 1) * POSITIONS_PER_TURN + targetPosition;

        int distance1 = Math.abs(target1 - currentValue);
        int distance2 = Math.abs(target2 - currentValue);

        // Determine which target to use based on constraints and distance
        if (target1 < MIN_POSITION) {
            // Can only go to target2
            setServoPosition(target2, targetPosition, 1);
        } else if (target2 > MAX_POSITION) {
            // Can only go to target1
            setServoPosition(target1, targetPosition, 0);
        } else {
            // Both targets valid, choose based on distance
            chooseOptimalTarget(target1, target2, distance1, distance2, targetPosition, 0, 1);
        }
    }

    /**
     * Choose the optimal target based on distances
     */
    private void chooseOptimalTarget(int target1, int target2, int distance1, int distance2,
                                     int targetPosition, int turnDelta1, int turnDelta2) {
        if (distance1 > distance2) {
            setServoPosition(target2, targetPosition, turnDelta2);
        } else if (distance2 > distance1) {
            setServoPosition(target1, targetPosition, turnDelta1);
        } else {
            // Distances are equal, choose based on which is closer to optimal distance
            int optimalDiff1 = Math.abs(distance1 - OPTIMAL_DISTANCE);
            int optimalDiff2 = Math.abs(distance2 - OPTIMAL_DISTANCE);

            if (optimalDiff1 > optimalDiff2) {
                setServoPosition(target2, targetPosition, turnDelta2);
            } else {
                setServoPosition(target1, targetPosition, turnDelta1);
            }
        }
    }

    /**
     * Set servo position and update state
     */
    private void setServoPosition(int targetValue, int targetPosition, int turnDelta) {
        double servoPosition = targetValue / SERVO_SCALE;

        // Clamp servo position to valid range [0.0, 1.0]
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        if (spin_dex != null) {
            spin_dex.setPosition(servoPosition);
        }

        currentPosition = targetPosition;
        currentTurn += turnDelta;
    }

    // ========== GETTER METHODS ==========

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int getCurrentTurn() {
        return currentTurn;
    }

    public String[] getSlots() {
        return slots.clone();
    }

    public String getSlotColor(int index) {
        if (index >= 0 && index < slots.length) {
            return slots[index];
        }
        return "unknown";
    }

    public double getServoPosition() {
        if (spin_dex != null) {
            return spin_dex.getPosition();
        }
        return -1.0;
    }

    // ========== SLOT MANAGEMENT ==========

    /**
     * Update the color of a specific slot (1-indexed to match position)
     */
    public void setSlotColor(int position, String color) {
        int slotIndex = position - 1; // Convert to 0-indexed
        if (slotIndex >= 0 && slotIndex < slots.length) {
            slots[slotIndex] = color;
        } else {
            telemetryM.debug("Error: Invalid slot position " + position);
        }
    }

    /**
     * Get color at current position
     */
    public String getCurrentSlotColor() {
        return getSlotColor(currentPosition - 1);
    }

    /**
     * Get count of filled slots
     */
    public int getFilledCount() {
        int count = 0;
        for (String slot : slots) {
            if (!"empty".equals(slot)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Get a formatted string of all slot states
     */
    public String getSlotsStatus() {
        StringBuilder status = new StringBuilder();
        for (int i = 0; i < slots.length; i++) {
            status.append("Slot ").append(i + 1).append(": ").append(slots[i]);
            if (i < slots.length - 1) status.append(", ");
        }
        return status.toString();
    }

    /**
     * Reset all slots to empty
     */
    public void clearAllSlots() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = "empty";
        }
    }

    // ========== COLOR SENSOR ==========

    /**
     * Check if color sensor is available
     */
    public boolean hasColorSensor() {
        return ballColorSensor != null;
    }

    /**
     * Get detected color (if sensor available)
     * Returns "green", "purple", "empty", or "No sensor"
     */
    public String getDetectedColor() {
        if (ballColorSensor == null) {
            return "No sensor";
        }

        try {
            // Read RGB values
            int red = ballColorSensor.red();
            int green = ballColorSensor.green();
            int blue = ballColorSensor.blue();

            // Detect green
            if (green > red && green > blue && green > GREEN_THRESHOLD) {
                return "green";
            }
            // Detect purple
            else if ((red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD)
                    && (red + blue > green * PURPLE_RATIO_THRESHOLD)) {
                return "purple";
            }
            // Nothing detected
            else {
                return "empty";
            }
        } catch (Exception e) {
            telemetryM.debug("Error reading color sensor: " + e.getMessage());
            return "error";
        }
    }

    /**
     * Get distance reading from color sensor (if available)
     * Returns distance in cm, or -1 if sensor unavailable
     */
    public double getDistance() {
        if (ballColorSensor != null) {
            try {
                return ballColorSensor.getDistance(DistanceUnit.CM);
            } catch (Exception e) {
                telemetryM.debug("Error reading distance: " + e.getMessage());
                return -1;
            }
        }
        return -1;
    }

    /**
     * Check if an artifact is present at current position (using distance sensor)
     */
    public boolean isArtifactPresent() {
        if (ballColorSensor == null) {
            return false;
        }
        double distance = getDistance();
        return distance > 0 && distance < 5.0; // Sample within 5cm
    }

    // ========== STATE & TELEMETRY ==========

    /**
     * Get SpinDex state as string for telemetry
     */
    public String getState() {
        return "Position " + currentPosition + "/" + POSITIONS_PER_TURN +
                " (Turn " + currentTurn + ")";
    }

    /**
     * Reset to home position
     */
    public void reset() {
        moveToPosition(1);
        currentTurn = 0;
    }
}



