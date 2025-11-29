package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {

    // --- LOGIC MAPS ---
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4};
    private static final int SHOOTING_OFFSET = 1;

    // --- HARDWARE CONSTANTS ---
    // If you ever change the servo programmer, update these.
    // Base 360 is the starting offset, 1620 is the total degrees range of the servo mapping?
    private static final double BASE_DEGREES = 360.0;
    private static final double DEGREES_PER_STEP = 60.0;
    private static final double DEGREES_PER_TURN = 360.0;
    private static final double TOTAL_RANGE_DIVISOR = 1620.0;

    public enum ArtifactType {
        EMPTY, GREEN, PURPLE
    }

    private final Servo spin_dex;
    private final Telemetry telemetry;
    private int currentPosition = 0;
    private final ArtifactType[] slots = new ArtifactType[3];

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spin_dex = hardwareMap.get(Servo.class, HW_SPINDEX);

        for (int i = 0; i < slots.length; i++) {
            slots[i] = ArtifactType.EMPTY;
        }
    }

    // --- MOVEMENT ---

    public void moveToPosition(int targetPosition) {
        // Clamp target to valid range 0-17 instead of modulo wrapping
        // logic to prevent accidental "snap backs"
        if (targetPosition < 0) targetPosition = 0;
        if (targetPosition > 17) targetPosition = 17;

        int turn = targetPosition / 6;
        int posInTurn = targetPosition % 6;

        // Hardware Calculation
        double servoPos = (BASE_DEGREES + (posInTurn * DEGREES_PER_STEP) + (turn * DEGREES_PER_TURN)) / TOTAL_RANGE_DIVISOR;

        // Safety check for array bounds
        if (posInTurn < OFFSETS.length) {
            servoPos += OFFSETS[posInTurn];
        }

        spin_dex.setPosition(servoPos);
        currentPosition = targetPosition;
    }

    // --- SMART SLOT SELECTION ---

    public boolean moveToNextEmptySlotForLoading() {
        int emptySlot = getNextEmptySlot();
        if (emptySlot != -1) {
            // Find the closest PHYSICAL instance of this logical slot
            moveToClosestSlotPosition(emptySlot, false);
            return true;
        }
        return false;
    }

    public boolean moveToNextFilledSlotForShooting() {
        int filledSlot = getClosestFilledSlot();
        if (filledSlot != -1) {
            moveToClosestSlotPosition(filledSlot, true);
            return true;
        }
        return false;
    }

    public boolean moveToPurpleArtifact() {
        int purpleSlot = getClosestPurpleSlot();
        if (purpleSlot != -1) {
            moveToClosestSlotPosition(purpleSlot, true);
            return true;
        }
        return false;
    }

    public boolean moveToGreenArtifact() {
        int greenSlot = getClosestGreenSlot();
        if (greenSlot != -1) {
            moveToClosestSlotPosition(greenSlot, true);
            return true;
        }
        return false;
    }

    // --- DISTANCE CALCULATION FIX ---

    private void moveToClosestSlotPosition(int slotIndex, boolean forShooting) {
        int mappedIndex = slotIndex % 3;
        int basePosInTurn = SLOT_TO_LOAD_POS_MAP[mappedIndex];

        if (forShooting) {
            basePosInTurn += SHOOTING_OFFSET;
        }

        // Calculate the physical position for this slot in all 3 turns
        int pos0 = basePosInTurn;           // Turn 0
        int pos1 = basePosInTurn + 6;       // Turn 1
        int pos2 = basePosInTurn + 12;      // Turn 2

        // Find which one is LINEARLY closest to currentPosition
        int targetPos = findClosestLinearPosition(currentPosition, pos0, pos1, pos2);

        moveToPosition(targetPos);
    }

    // REPLACED: This now uses Math.abs for linear distance on a Servo
    private int findClosestLinearPosition(int current, int p0, int p1, int p2) {
        int dist0 = Math.abs(current - p0);
        int dist1 = Math.abs(current - p1);
        int dist2 = Math.abs(current - p2);

        if (dist0 <= dist1 && dist0 <= dist2) return p0;
        else if (dist1 <= dist2) return p1;
        else return p2;
    }

    // FIXED: Iterates all Filled slots and finds the absolute closest one
    private int getClosestFilledSlot() {
        int bestSlot = -1;
        int minDistance = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            if (slots[i] != ArtifactType.EMPTY) {
                // Where would this slot be in all 3 turns?
                int base = SLOT_TO_LOAD_POS_MAP[i] + SHOOTING_OFFSET;

                int dist0 = Math.abs(currentPosition - base);
                int dist1 = Math.abs(currentPosition - (base + 6));
                int dist2 = Math.abs(currentPosition - (base + 12));

                int localMin = Math.min(dist0, Math.min(dist1, dist2));

                if (localMin < minDistance) {
                    minDistance = localMin;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    private int getClosestPurpleSlot() {
        int bestSlot = -1;
        int minDistance = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            if (slots[i] == ArtifactType.PURPLE) {
                int base = SLOT_TO_LOAD_POS_MAP[i] + SHOOTING_OFFSET;
                int dist0 = Math.abs(currentPosition - base);
                int dist1 = Math.abs(currentPosition - (base + 6));
                int dist2 = Math.abs(currentPosition - (base + 12));
                int localMin = Math.min(dist0, Math.min(dist1, dist2));

                if (localMin < minDistance) {
                    minDistance = localMin;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    private int getClosestGreenSlot() {
        // Same logic as Purple
        int bestSlot = -1;
        int minDistance = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            if (slots[i] == ArtifactType.GREEN) {
                int base = SLOT_TO_LOAD_POS_MAP[i] + SHOOTING_OFFSET;
                int dist0 = Math.abs(currentPosition - base);
                int dist1 = Math.abs(currentPosition - (base + 6));
                int dist2 = Math.abs(currentPosition - (base + 12));
                int localMin = Math.min(dist0, Math.min(dist1, dist2));

                if (localMin < minDistance) {
                    minDistance = localMin;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    // --- STATE HELPERS ---

    public ArtifactType getSlot(int index) {
        return slots[index % 3];
    }

    // Sets the type of artifact currently held in the specified logical slot.
    // Use this after a successful intake.
    public void setSlot(int index, ArtifactType type) {
        slots[index % 3] = type;
    }

    // Clears the specified logical slot.
    // Use this after a successful shot.
    public void clearSlot(int index) {
        slots[index % 3] = ArtifactType.EMPTY;
    }

    // Clears all slots - resets the tracking array
    public void clearAllSlots() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = ArtifactType.EMPTY;
        }
    }

    public int getFilledCount() {
        int count = 0;
        for (ArtifactType s : slots)
            if (s != ArtifactType.EMPTY) count++;
        return count;
    }

    public int getNextEmptySlot() {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == ArtifactType.EMPTY) return i;
        }
        return -1;
    }

    public boolean isFull() {
        return getFilledCount() == 3;
    }

    public boolean isEmpty() {
        return getFilledCount() == 0;
    }

    // GETTERS

    public int getCurrentPosition() {
        return currentPosition;
    }

    public double getServoPosition() {
        return spin_dex.getPosition();
    }

    public int getCurrentTurn() {
        return currentPosition / 6;
    }
}