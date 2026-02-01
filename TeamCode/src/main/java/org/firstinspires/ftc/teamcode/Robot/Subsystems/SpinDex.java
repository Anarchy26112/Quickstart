package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.*;


import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    // --- LOGIC MAPS ---
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4}; // Base positions within one full rotation
    private static final int SHOOTING_OFFSET = 3;

    // --- MOTOR/ENCODER CONSTANTS ---
    private static final double MOTOR_PPR = 384.5;               // goBILDA 435 RPM yellow jacket motor
    private static final double POSITIONS_PER_REVOLUTION = 6.0;  // 360° / 60°
    private static final double TICKS_PER_POSITION = MOTOR_PPR / POSITIONS_PER_REVOLUTION;

    // --- TUNING ---
    public static final double POSITION_TOLERANCE_TICKS = 0.5;  // deadband around target
    private static final double MAX_POWER = 1.0;

    public enum ArtifactType { EMPTY, GREEN, PURPLE }

    private final DcMotorEx spinDexMotor;
    private final Telemetry telemetry;

    private int currentPositionIndex = 0;     // estimated discrete index (infinite)
    private int targetPositionTicks = 0;      // absolute encoder ticks target

    private final ArtifactType[] slots = new ArtifactType[3];

    private boolean diditwork = false;

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spinDexMotor = hardwareMap.get(DcMotorEx.class, "spindexmotor");

        spinDexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Use encoder-based modes (needed for PIDF and RUN_TO_POSITION)
        spinDexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(
                DEFAULT_KP, 0,
                DEFAULT_KD, 0
        );
        spinDexMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Init slot state
        for (int i = 0; i < slots.length; i++) slots[i] = ArtifactType.EMPTY;

        // Initialize target to current position (avoid surprise motion)
        targetPositionTicks = spinDexMotor.getCurrentPosition();
    }

    public void periodic() {
        int currentTicks = spinDexMotor.getCurrentPosition();
        currentPositionIndex = (int) Math.round(currentTicks / TICKS_PER_POSITION);

        int error = targetPositionTicks - currentTicks;

        // If we're basically there, stop driving
        if (Math.abs(error) <= POSITION_TOLERANCE_TICKS) {
            spinDexMotor.setPower(0);

            if (spinDexMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                spinDexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return;
        }

        // Ensure correct mode
        if (spinDexMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spinDexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // 🔑 ALWAYS refresh target (cheap + safe)
        spinDexMotor.setTargetPosition(targetPositionTicks);

        // Drive
        spinDexMotor.setPower(MAX_POWER);
    }

    private void setTargetTicks(double newTargetTicks) {
        targetPositionTicks = (int) Math.round(newTargetTicks);
    }


    // --- CORE MOVEMENT: Move to the closest instance of a given base position ---
    private void moveToClosestPosition(int basePosition, boolean forShooting) {
        int offsetBase = forShooting ? basePosition + SHOOTING_OFFSET : basePosition;

        // Find nearest multiple of 6 away from offset base
        int revolutionOffset = (int) Math.round((double) (currentPositionIndex - offsetBase) / POSITIONS_PER_REVOLUTION);
        int closest = offsetBase + revolutionOffset * (int) POSITIONS_PER_REVOLUTION;

        // Check +/- one revolution for true shortest path
        int candidate1 = closest + (int) POSITIONS_PER_REVOLUTION;
        int candidate2 = closest - (int) POSITIONS_PER_REVOLUTION;

        int best = closest;
        int minDist = Math.abs(currentPositionIndex - closest);

        int dist1 = Math.abs(currentPositionIndex - candidate1);
        if (dist1 < minDist) {
            minDist = dist1;
            best = candidate1;
        }

        int dist2 = Math.abs(currentPositionIndex - candidate2);
        if (dist2 < minDist) {
            best = candidate2;
        }

        setTargetTicks(best * TICKS_PER_POSITION);
    }

    // --- MANUAL MOVEMENT (for operator controls) ---
    public void moveToPosition(int targetIndex) {
        setTargetTicks(targetIndex * TICKS_PER_POSITION);
    }

    // --- SMART SLOT SELECTION ---
    public boolean moveToNextEmptySlotForLoading() {
        int emptySlot = getNextEmptySlot();
        if (emptySlot == -1) return false;

        int basePos = SLOT_TO_LOAD_POS_MAP[emptySlot];
        moveToClosestPosition(basePos, false);
        return true;
    }

    public boolean moveToNextFilledSlotForShooting() {
        int filledSlot = getClosestFilledSlot();
        if (filledSlot == -1) return false;

        int basePos = SLOT_TO_LOAD_POS_MAP[filledSlot];
        moveToClosestPosition(basePos, true);
        return true;
    }

    public boolean moveToPurpleArtifact() {
        int purpleSlot = getClosestPurpleSlot();
        if (purpleSlot == -1) return false;

        int basePos = SLOT_TO_LOAD_POS_MAP[purpleSlot];
        moveToClosestPosition(basePos, true);
        return true;
    }

    public boolean moveToGreenArtifact() {
        int greenSlot = getClosestGreenSlot();
        if (greenSlot == -1) return false;

        int basePos = SLOT_TO_LOAD_POS_MAP[greenSlot];
        moveToClosestPosition(basePos, true);
        return true;
    }

    // --- SLOT DISTANCE HELPERS (infinite range) ---
    private int getClosestFilledSlot() {
        return getClosestSlotOfType(type -> type != ArtifactType.EMPTY);
    }

    private int getClosestPurpleSlot() {
        return getClosestSlotOfType(type -> type == ArtifactType.PURPLE);
    }

    private int getClosestGreenSlot() {
        return getClosestSlotOfType(type -> type == ArtifactType.GREEN);
    }

    private interface SlotFilter {
        boolean matches(ArtifactType type);
    }

    private int getClosestSlotOfType(SlotFilter filter) {
        int bestSlot = -1;
        int minSteps = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            if (filter.matches(slots[i])) {
                int base = SLOT_TO_LOAD_POS_MAP[i] + SHOOTING_OFFSET;

                int revOffset = (int) Math.round((double) (currentPositionIndex - base) / POSITIONS_PER_REVOLUTION);

                int d0 = Math.abs(currentPositionIndex - (base + revOffset * (int) POSITIONS_PER_REVOLUTION));
                int d1 = Math.abs(currentPositionIndex - (base + (revOffset + 1) * (int) POSITIONS_PER_REVOLUTION));
                int d2 = Math.abs(currentPositionIndex - (base + (revOffset - 1) * (int) POSITIONS_PER_REVOLUTION));
                int localMin = Math.min(d0, Math.min(d1, d2));

                if (localMin < minSteps) {
                    minSteps = localMin;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    // --- STATE HELPERS ---
    public ArtifactType getSlot(int index) { return slots[index % 3]; }
    public void setSlot(int index, ArtifactType type) { slots[index % 3] = type; }
    public void clearSlot(int index) { slots[index % 3] = ArtifactType.EMPTY; }
    public void clearAllSlots() { for (int i = 0; i < slots.length; i++) slots[i] = ArtifactType.EMPTY; }

    public int getFilledCount() {
        int count = 0;
        for (ArtifactType s : slots) if (s != ArtifactType.EMPTY) count++;
        return count;
    }

    public int getNextEmptySlot() {
        for (int i = 0; i < 3; i++) if (slots[i] == ArtifactType.EMPTY) return i;
        return -1;
    }

    public boolean isFull() { return getFilledCount() == 3; }
    public boolean isEmpty() { return getFilledCount() == 0; }

    public boolean hasTwoPurplesOneGreen() {
        int purple = 0, green = 0;
        for (ArtifactType s : slots) {
            if (s == ArtifactType.PURPLE) purple++;
            else if (s == ArtifactType.GREEN) green++;
        }
        return purple == 2 && green == 1;
    }

    public boolean isAtTarget() {
        return Math.abs(targetPositionTicks - spinDexMotor.getCurrentPosition()) <= POSITION_TOLERANCE_TICKS;
    }

    // --- GETTERS ---
    public int getCurrentPosition() { return currentPositionIndex; }
    public int getCurrentPositionIndex() { return currentPositionIndex; }
    public int getMotorPosition() { return spinDexMotor.getCurrentPosition(); }
    public double getTargetPositionTicks() { return targetPositionTicks; }
    public int getCurrentTurn() { return currentPositionIndex / 6; }
    public double getServoPosition() { return targetPositionTicks; } // telemetry compatibility
    public boolean getdiditwork() { return diditwork; }

    // --- PIDF TUNING (like your example) ---
    public void setPIDFGains(double p, double i, double d, double f) {
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        spinDexMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void resetEncoder() {
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPositionIndex = 0;
        targetPositionTicks = 0;

        // Stop any motion
        spinDexMotor.setPower(0);
    }

    // For Shooter Macro

    public int getClosestFilledSlotIndexForShooting() {
        // reuses your existing private logic
        return getClosestFilledSlot();
    }

    public boolean moveToSpecificSlotForShooting(int slotIndex) {
        if (slotIndex < 0 || slotIndex > 2) return false;
        if (slots[slotIndex] == ArtifactType.EMPTY) return false;

        int basePos = SLOT_TO_LOAD_POS_MAP[slotIndex];
        moveToClosestPosition(basePos, true);
        return true;
    }

}

/*
public void periodic() {
    int currentTicks = spinDexMotor.getCurrentPosition();
    currentPositionIndex = (int) Math.round(currentTicks / TICKS_PER_POSITION);

    // Always stay in RUN_TO_POSITION
    if (spinDexMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
        spinDexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    spinDexMotor.setTargetPosition(targetPositionTicks);

    int error = targetPositionTicks - currentTicks;
    if (Math.abs(error) <= POSITION_TOLERANCE_TICKS) {
        // Either hold with small power or zero depending on your mechanism
        spinDexMotor.setPower(0.0);
    } else {
        spinDexMotor.setPower(MAX_POWER);
    }
}
 */