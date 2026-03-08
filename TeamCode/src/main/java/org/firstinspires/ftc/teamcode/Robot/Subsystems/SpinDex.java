package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.CloseBlueAuto;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    // --- LOGIC MAPS ---
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4}; // Base positions within one full rotation
    private static final int SHOOTING_OFFSET = 3;

    // --- MOTOR/ENCODER CONSTANTS ---
    private static final double MOTOR_PPR = 384.5;               // goBILDA 435 RPM yellow jacket motor
    private static final double POSITIONS_PER_REVOLUTION = 6.0;  // 360° / 60°
    private static final double TICKS_PER_POSITION = MOTOR_PPR / POSITIONS_PER_REVOLUTION;
    private static double TICKS_PER_REV = MOTOR_PPR;
    private static double TICKS_PER_POS = TICKS_PER_REV / POSITIONS_PER_REVOLUTION;


    // --- CONTROL LIMITS ---
    public static final double POSITION_TOLERANCE_TICKS = 2.0; // deadband around target
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1.0;

    // Optional: minimum power to overcome stiction (set 0 if not needed)
    private static final double STATIC_FF = 0.00;

    public enum ArtifactType { EMPTY, GREEN, PURPLE }

    private final DcMotorEx spinDexMotor;
    private final Telemetry telemetry;

    private int currentPositionIndex = 0; // estimated discrete index (infinite)
    private int targetPositionTicks = 0;  // absolute encoder ticks target

    private final ArtifactType[] slots = new ArtifactType[3];

    private final ElapsedTime pdTimer = new ElapsedTime();
    private double lastError = 0;
    private DcMotor.RunMode internalModeCache = null;

    // --- MICRO ADJUST / REZERO ---
    private static final int MICRO_ADJUST_TICKS = 16;
    private boolean pendingRezero = false;

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spinDexMotor = hardwareMap.get(DcMotorEx.class, "spindexmotor");

        spinDexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ✅ If Auto saved ticks, DO NOT reset encoder; reuse position
        if (SpinDexHandoff.hasSaved()) {
            // Keep current encoder reading (no reset)
            spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set target to the saved absolute ticks so PD holds that position
            targetPositionTicks = SpinDexHandoff.getSavedTicks();

        } else {
            // Default behavior on fresh start / no handoff available
            spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            targetPositionTicks = spinDexMotor.getCurrentPosition();
        }

        // Init slot state
        for (int i = 0; i < slots.length; i++) slots[i] = ArtifactType.EMPTY;

        // Initialize PD timer state
        pdTimer.reset();
        lastError = 0;
    }

    public void periodic() {
        // Ensure correct mode
        if (internalModeCache != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            internalModeCache = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }

        int currentTicks = spinDexMotor.getCurrentPosition();
        currentPositionIndex = (int) Math.round(currentTicks / TICKS_PER_POS);

        double error = targetPositionTicks - currentTicks;

        // Deadband: stop driving if we're close enough
        if (Math.abs(error) <= POSITION_TOLERANCE_TICKS) {
            spinDexMotor.setPower(0);

            // If a micro-adjust requested rezero, do it once we arrive
            if (pendingRezero) {
                performRezeroHere();
                return;
            }

            lastError = error;
            pdTimer.reset();
            return;
        }

        double dt = pdTimer.seconds();
        pdTimer.reset();
        if (dt <= 0) dt = 1e-3;

        double derivative = (error - lastError) / dt;

        double output = (DEFAULT_KP * error) + (DEFAULT_KD * derivative);

        output = clamp(output, MIN_POWER, MAX_POWER);

        spinDexMotor.setPower(output);
        lastError = error;
    }

    /**
     * Sets a new target in encoder ticks.
     * (IMPROVEMENT #2) Resets derivative memory to avoid derivative kick when targets jump.
     */
    private void setTargetTicks(double newTargetTicks) {
        targetPositionTicks = (int) Math.round(newTargetTicks);

        // --- #2: Prevent derivative kick on a target change ---
        int currentTicks = spinDexMotor.getCurrentPosition();
        double newError = targetPositionTicks - currentTicks;
        lastError = newError;
        pdTimer.reset();
    }
    private static double wrapNearest(double baseTicks, double currentTicks, double ticksPerRev) {
        // returns the equivalent of baseTicks + k*ticksPerRev that is nearest to currentTicks
        double k = Math.rint((currentTicks - baseTicks) / ticksPerRev); // rint => nearest integer as double
        return baseTicks + k * ticksPerRev;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // --- CORE MOVEMENT: Move to the closest instance of a given base position (TICK-SPACE) ---
    private void moveToClosestPosition(int basePosition, boolean forShooting) {
        int offsetBaseIndex = forShooting ? basePosition + SHOOTING_OFFSET : basePosition;

        // Convert base "position index" into base ticks within a revolution
        double baseTicks = offsetBaseIndex * TICKS_PER_POS;

        double currentTicks = spinDexMotor.getCurrentPosition();

        // Find nearest equivalent target ticks (allowing wrap by +/- revolutions)
        double nearest = wrapNearest(baseTicks, currentTicks, TICKS_PER_REV);

        // Check +/- one revolution too (for safety if you're near halfway)
        double cand0 = nearest;
        double cand1 = nearest + TICKS_PER_REV;
        double cand2 = nearest - TICKS_PER_REV;

        double best = cand0;
        double bestDist = Math.abs(currentTicks - cand0);

        double d1 = Math.abs(currentTicks - cand1);
        if (d1 < bestDist) { bestDist = d1; best = cand1; }

        double d2 = Math.abs(currentTicks - cand2);
        if (d2 < bestDist) { best = cand2; }

        setTargetTicks(best);
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
        double bestDistTicks = Double.POSITIVE_INFINITY;

        double currentTicks = spinDexMotor.getCurrentPosition();

        for (int i = 0; i < 3; i++) {
            if (!filter.matches(slots[i])) continue;

            int baseIndex = SLOT_TO_LOAD_POS_MAP[i] + SHOOTING_OFFSET; // shooting alignment
            double baseTicks = baseIndex * TICKS_PER_POS;

            double nearest = wrapNearest(baseTicks, currentTicks, TICKS_PER_REV);

            // also consider +/- one rev (same idea as moveToClosestPosition)
            double cand0 = nearest;
            double cand1 = nearest + TICKS_PER_REV;
            double cand2 = nearest - TICKS_PER_REV;

            double localBest = cand0;
            double localDist = Math.abs(currentTicks - cand0);

            double d1 = Math.abs(currentTicks - cand1);
            if (d1 < localDist) { localDist = d1; localBest = cand1; }

            double d2 = Math.abs(currentTicks - cand2);
            if (d2 < localDist) { localDist = d2; localBest = cand2; }

            if (localDist < bestDistTicks) {
                bestDistTicks = localDist;
                bestSlot = i;
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
        return Math.abs(targetPositionTicks - spinDexMotor.getCurrentPosition()) <= 8;
    }

    // --- GETTERS ---
    public int getCurrentPosition() { return currentPositionIndex; }
    public int getCurrentPositionIndex() { return currentPositionIndex; }
    public int getMotorPosition() { return spinDexMotor.getCurrentPosition(); }
    public double getTargetPositionTicks() { return targetPositionTicks; }
    public int getCurrentTurn() { return currentPositionIndex / 6; }
    public double getServoPosition() { return targetPositionTicks; } // telemetry compatibility

    // --- RESET / TUNING ---
    public void resetEncoder() {
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // --- #1: Keep mode consistent with periodic() (custom PD power control) ---
        spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentPositionIndex = 0;
        targetPositionTicks = 0;

        // Reset PD memory too
        lastError = 0;
        pdTimer.reset();

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
    public void microAdjustRightAndRezero() {
        setTargetTicks(spinDexMotor.getCurrentPosition() + MICRO_ADJUST_TICKS);
        pendingRezero = true;
    }

    public void microAdjustLeftAndRezero() {
        setTargetTicks(spinDexMotor.getCurrentPosition() - MICRO_ADJUST_TICKS);
        pendingRezero = true;
    }

    private void performRezeroHere() {
        spinDexMotor.setPower(0);
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentPositionIndex = 0;
        targetPositionTicks = 0;
        lastError = 0;
        pdTimer.reset();
        pendingRezero = false;
    }
}
