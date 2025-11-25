package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {

    // CONFIGURATION CONSTANTS (UPDATED)
    // Defines the physical position (0, 2, or 4) that corresponds to the
    // LOADING/INTAKE alignment for each logical slot (0, 1, 2).
    // Mapping: Slot 0 -> Pos 0, Slot 1 -> Pos 2, Slot 2 -> Pos 4
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4};

    // Defines the offset from the Loading Position to the Shooting Position (1, 3, 5).
    // E.g., Pos 0 (Load) + 1 = Pos 1 (Shoot)
    private static final int SHOOTING_OFFSET = 1;

    // ENUM FOR ARTIFACT TRACKING
    public enum ArtifactType {
        EMPTY,
        GREEN,
        PURPLE
    }

    // FIELDS
    private final Servo spin_dex;
    private final Telemetry telemetry;
    private int currentPosition = 0; // The current physical position (0-5)
    private final ArtifactType[] slots = new ArtifactType[3]; // The logical state of the 3 slots

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // The HW_SPINDEX constant must be defined in HamiltonParams
        this.spin_dex = hardwareMap.get(Servo.class, HW_SPINDEX);

        // Initialize all slots as empty upon startup
        for (int i = 0; i < slots.length; i++) {
            slots[i] = ArtifactType.EMPTY;
        }
    }

    // POSITION CONTROL (Physical Movement)

    // Moves the servo to a specific physical position (0-5). This method handles
    // the wrapping/modulo and applies hardware specific offsets.
    public void moveToPosition(int targetPosition) {
        // Ensure the target is always between 0 and 5
        targetPosition = ((targetPosition % 6) + 6) % 6;

        // Calculate the raw servo position (assuming 6 positions spread over the servo range)
        double servoPos = (720.0 + (targetPosition * 60.0)) / 1620.0;

        // Apply any position-specific offsets defined in HamiltonParams
        // The OFFSETS array must be defined and have 6 elements.
        servoPos += OFFSETS[targetPosition];

        spin_dex.setPosition(servoPos);

        currentPosition = targetPosition;
    }

    public void rotateCW() {
        // Rotates to the next physical position
        moveToPosition(currentPosition + 1);
    }

    public void rotateCCW() {
        // Rotates to the previous physical position
        moveToPosition(currentPosition - 1);
    }

    // TARGETING (Logical Movement)

    // Moves the Spindex to align a specific logical slot (0, 1, or 2) for action.
    // This method translates the logical slot index into the correct physical position (0-5).
    public void moveToSlot(int slotIndex, boolean forShooting) {
        // Ensure slotIndex is valid (0, 1, or 2)
        int mappedIndex = slotIndex % 3;

        // 1. Get the base position for loading (0, 2, or 4)
        int targetPos = SLOT_TO_LOAD_POS_MAP[mappedIndex];

        // 2. Adjust position if we are aiming for the shooting spot (1, 3, or 5)
        if (forShooting) {
            targetPos += SHOOTING_OFFSET;
        }

        // 3. Move to the calculated physical position (moveToPosition handles the modulo 6 wrap)
        moveToPosition(targetPos);
    }

    // Automatically finds the next empty slot and moves the Spindex to its loading position (0, 2, or 4).
    public boolean moveToNextEmptySlotForLoading() {
        int emptySlot = getNextEmptySlot();
        if (emptySlot != -1) {
            // Move to the loading position (forShooting = false)
            moveToSlot(emptySlot, false);
            return true;
        }
        return false;
    }
    // Automatically finds the next filled slot and moves the Spindex to its shooting position (1, 3, or 5).
    public boolean moveToNextFilledSlotForShooting() {
        int filledSlot = getNextFilledSlot();
        if (filledSlot != -1) {
            // Move to the shooting position (forShooting = true)
            moveToSlot(filledSlot, true);
            return true;
        }
        return false;
    }

    // Automatically finds the next purple artifact and moves the Spindex to its shooting position (1, 3, or 5).
    public boolean moveToPurpleArtifact() {
        int purpleSlot = getNextPurpleSlot();
        if (purpleSlot != -1) {
            // Move to the shooting position (forShooting = true)
            moveToSlot(purpleSlot, true);
            return true;
        }
        return false;
    }

    // Automatically finds the next green artifact and moves the Spindex to its shooting position (1, 3, or 5).
    public boolean moveToGreenArtifact() {
        int greenSlot = getNextGreenSlot();
        if (greenSlot != -1) {
            // Move to the shooting position (forShooting = true)
            moveToSlot(greenSlot, true);
            return true;
        }
        return false;
    }

    // SLOT TRACKING

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

    public int getNextFilledSlot() {
        for (int i = 0; i < 3; i++) {
            if (slots[i] != ArtifactType.EMPTY) return i;
        }
        return -1;
    }

    public int getNextPurpleSlot() {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == ArtifactType.PURPLE) return i;
        }
        return -1;
    }

    public int getNextGreenSlot() {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == ArtifactType.GREEN) return i;
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
        return 0;
    }
}