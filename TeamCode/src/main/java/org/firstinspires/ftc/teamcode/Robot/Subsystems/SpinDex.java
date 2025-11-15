package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    private final Servo spin_dex;
    private final Telemetry telemetry;

    // State tracking
    public String[] slots = {"empty", "empty", "empty"}; // 3 slots
    private int currentPosition = 1; // Start at home
    private int currentTurn = 0;


    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize servo
        spin_dex = hardwareMap.get(Servo.class, HW_SPINDEX);
    }

    // Move to a target position (1-6)
    public void moveToPosition(int targetPosition) {
        if (targetPosition < 1 || targetPosition > POSITIONS_PER_TURN) {
            telemetry.addData("Error", "Invalid position " + targetPosition + " (valid range: 1-" + POSITIONS_PER_TURN + ")");
            return;
        }

        if (spin_dex == null) {
            telemetry.addData("Error", "Cannot move - servo not initialized");
            return;
        }

        if (currentPosition < targetPosition) {
            sortBallsForward(targetPosition);
        } else if (currentPosition > targetPosition) {
            sortBallsBackward(targetPosition);
        }
        // If equal, do nothing
    }

    // Rotate forward to target position
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

    // Rotate backward to target position
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

    // Choose the optimal target based on distances
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

    // Set servo position and update state
    private void setServoPosition(int targetValue, int targetPosition, int turnDelta) {
        double servoPosition = targetValue / SERVO_SCALE;

        // Clamp servo position to valid range [0.0, 1.0]
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition+OFFSET));

        if (spin_dex != null) {
            spin_dex.setPosition(servoPosition+OFFSET);
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

    // Get color at current position
    public String getCurrentSlotColor() {
        return getSlotColor(currentPosition - 1);
    }

    // Get count of filled slots
    public int getFilledCount() {
        int count = 0;
        for (String slot : slots) {
            if (!"empty".equals(slot)) {
                count++;
            }
        }
        return count;
    }

    // Reset all slots to empty
    public void clearAllSlots() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = "empty";
        }
    }

    // ========== STATE & TELEMETRY ==========

    // Get SpinDex state as string for telemetry
    public String getState() {
        return "Position " + currentPosition + "/" + POSITIONS_PER_TURN +
                " (Turn " + currentTurn + ")";
    }

    // Reset to home position
    public void reset() {
        moveToPosition(1);
        currentTurn = 0;
    }
}