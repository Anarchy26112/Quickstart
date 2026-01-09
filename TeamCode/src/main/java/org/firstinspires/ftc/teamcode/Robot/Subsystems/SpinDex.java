package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    // --- LOGIC MAPS ---
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4}; // Base positions within one full rotation
    private static final int SHOOTING_OFFSET = 3;

    // --- MOTOR CONSTANTS ---
    private static final double MOTOR_PPR = 384.5; // goBILDA 435 RPM yellow jacket motor
    private static final double DEGREES_PER_STEP = 60.0;
    private static final double POSITIONS_PER_REVOLUTION = 6.0; // 360° / 60°

    private static final double TICKS_PER_POSITION = MOTOR_PPR / POSITIONS_PER_REVOLUTION;

    // --- PD CONTROLLER ---
    private static class PDController {
        private double Kp, Kd;
        private double lastPosition = 0;
        private boolean firstRun = true;
        private final ElapsedTime timer;

        public PDController(double Kp, double Kd) {
            this.Kp = Kp;
            this.Kd = Kd;
            this.timer = new ElapsedTime();
        }

        public double update(double target, double state) {
            double dt = timer.seconds();
            timer.reset();

            // If dt is invalid or too large (controller paused), fall back to P-only
            if (dt <= 0 || dt > 1.0) {
                lastPosition = state;
                firstRun = false;
                return Kp * (target - state);
            }

            double error = target - state;

            double derivative = 0;
            if (!firstRun) {
                // derivative of error ~= -velocity
                derivative = -(state - lastPosition) / dt;
            }

            lastPosition = state;
            firstRun = false;

            return Kp * error + Kd * derivative;
        }

        /** Reset the controller, anchoring history to the current measured state. */
        public void resetTo(double state) {
            lastPosition = state;
            firstRun = true;
            timer.reset();
        }

        // Kept for compatibility if anything else calls it
        public void reset() {
            lastPosition = 0;
            firstRun = true;
            timer.reset();
        }

        public void setGains(double Kp, double Kd) {
            this.Kp = Kp;
            this.Kd = Kd;
        }
    }

    public enum ArtifactType {
        EMPTY, GREEN, PURPLE
    }

    private final DcMotorEx spinDexMotor;
    private final Telemetry telemetry;
    private final PDController pdController;

    private int currentPositionIndex = 0;        // Current estimated position index (infinite)
    private double targetPositionTicks = 0;       // Target in encoder ticks

    private final ArtifactType[] slots = new ArtifactType[3];

    private static final double POSITION_TOLERANCE = 3.0; // ticks
    private static final double MAX_POWER = 1.0;

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spinDexMotor = hardwareMap.get(DcMotorEx.class, "spindexmotor");

        spinDexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pdController = new PDController(DEFAULT_KP, DEFAULT_KD);

        for (int i = 0; i < slots.length; i++) {
            slots[i] = ArtifactType.EMPTY;
        }
    }

    public void periodic() {
        double currentTicks = spinDexMotor.getCurrentPosition();

        // Update our logical position index for use in shortest-path calculations
        currentPositionIndex = (int) Math.round(currentTicks / TICKS_PER_POSITION);

        double error = targetPositionTicks - currentTicks;

        // Deadband: if we're close enough, stop driving to prevent hunting
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            spinDexMotor.setPower(0);

            pdController.resetTo(currentTicks);
            return;
        }

        double command = pdController.update(targetPositionTicks, currentTicks);

        command = Math.max(-MAX_POWER, Math.min(MAX_POWER, command));

        // Enforce minimum power when we still have meaningful error
        if (Math.abs(error) > POSITION_TOLERANCE) {
            if (Math.abs(command) < SPINDEX_MIN_POWER) {
                command = Math.copySign(SPINDEX_MIN_POWER, command);
            }
        }

        spinDexMotor.setPower(command);
    }

    private void setTargetTicks(double newTargetTicks) {
        targetPositionTicks = newTargetTicks;

        // (1) Reset PD anchored to current state to avoid any derivative/history weirdness
        pdController.resetTo(spinDexMotor.getCurrentPosition());
    }

    // --- CORE MOVEMENT: Move to the closest instance of a given base position ---
    private void moveToClosestPosition(int basePosition, boolean forShooting) {
        int offsetBase = forShooting ? basePosition + SHOOTING_OFFSET : basePosition;

        // Find the nearest multiple of 6 positions away from the offset base
        int revolutionOffset = (int) Math.round((double) (currentPositionIndex - offsetBase) / POSITIONS_PER_REVOLUTION);
        int closest = offsetBase + revolutionOffset * (int) POSITIONS_PER_REVOLUTION;

        // Also check one revolution in each direction to ensure true shortest path
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

        // Set target ticks using helper (resets PD anchored to current ticks)
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

    // --- SLOT DISTANCE HELPERS (now use infinite range) ---
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
                int closest = Math.abs(currentPositionIndex - (base + revOffset * (int) POSITIONS_PER_REVOLUTION));

                int distPlus = Math.abs(currentPositionIndex - (base + (revOffset + 1) * (int) POSITIONS_PER_REVOLUTION));
                int distMinus = Math.abs(currentPositionIndex - (base + (revOffset - 1) * (int) POSITIONS_PER_REVOLUTION));
                int localMin = Math.min(closest, Math.min(distPlus, distMinus));

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
        return Math.abs(targetPositionTicks - spinDexMotor.getCurrentPosition()) <= POSITION_TOLERANCE;
    }

    // --- GETTERS ---
    public int getCurrentPosition() { return currentPositionIndex; }
    public int getCurrentPositionIndex() { return currentPositionIndex; }
    public int getMotorPosition() { return spinDexMotor.getCurrentPosition(); }
    public double getTargetPositionTicks() { return targetPositionTicks; }
    public int getCurrentTurn() { return currentPositionIndex / 6; }
    public double getServoPosition() { return targetPositionTicks; } // For telemetry compatibility

    // --- TUNING ---
    public void setPDGains(double Kp, double Kd) { pdController.setGains(Kp, Kd); }

    public void resetEncoder() {
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentPositionIndex = 0;
        targetPositionTicks = 0;

        // Reset anchored to the new encoder reading (0)
        pdController.resetTo(spinDexMotor.getCurrentPosition());
    }
}
