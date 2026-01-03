package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class SpinDex {
    // --- LOGIC MAPS ---
    private static final int[] SLOT_TO_LOAD_POS_MAP = {0, 2, 4}; // Base positions within one full rotation
    private static final int SHOOTING_OFFSET = 3;

    // --- MOTOR CONSTANTS ---
    private static final double MOTOR_PPR = 384.5; // goBILDA 435 RPM motor
    private static final double POSITIONS_PER_REVOLUTION = 6.0; // 360° / 60°
    private static final double TICKS_PER_POSITION = MOTOR_PPR / POSITIONS_PER_REVOLUTION; // ≈89.6167 ticks per 60°

    public enum ArtifactType {
        EMPTY, GREEN, PURPLE
    }

    private final DcMotorEx spinDexMotor;
    private final Telemetry telemetry;

    private int currentPositionIndex = 0;     // Current estimated position index (infinite)
    private double targetPositionTicks = 0;   // Target in encoder ticks (double for telemetry)

    private final ArtifactType[] slots = new ArtifactType[3];

    private static final int POSITION_TOLERANCE_TICKS = 5; // ticks
    private static final double MAX_POWER = 1.0;

    // Store gains so you can change them anytime
    private double kp = DEFAULT_KP;
    private double kd = DEFAULT_KD;

    public SpinDex(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.spinDexMotor = hardwareMap.get(DcMotorEx.class, "spindexmotor");

        spinDexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder, then use built-in controller
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinDexMotor.setTargetPosition(0);
        spinDexMotor.setTargetPositionTolerance(POSITION_TOLERANCE_TICKS);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply initial PD gains (I=0, F=0)
        applyBuiltInPD(kp, kd);

        // Start with no motion until you command a target
        spinDexMotor.setPower(0);

        for (int i = 0; i < slots.length; i++) {
            slots[i] = ArtifactType.EMPTY;
        }
    }

    public void periodic() {
        double currentTicks = spinDexMotor.getCurrentPosition();
        currentPositionIndex = (int) Math.round(currentTicks / TICKS_PER_POSITION);

        // RUN_TO_POSITION handles control internally.
        // You can add telemetry here if you want.
        // telemetry.addData("SpinDex ticks", currentTicks);
        // telemetry.addData("Target ticks", targetPositionTicks);
        // telemetry.addData("Busy", spinDexMotor.isBusy());
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

        // Set target ticks and let RUN_TO_POSITION do the work
        setTargetTicks(best * TICKS_PER_POSITION);
    }

    private void setTargetTicks(double ticks) {
        targetPositionTicks = ticks;

        // RUN_TO_POSITION needs an int target
        spinDexMotor.setTargetPosition((int) Math.round(targetPositionTicks));

        // Power is the max drive power used by the internal controller
        spinDexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinDexMotor.setPower(MAX_POWER);
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

    public boolean isAtTarget() {
        return Math.abs(targetPositionTicks - spinDexMotor.getCurrentPosition()) <= POSITION_TOLERANCE_TICKS;
    }

    // --- GETTERS ---
    public int getCurrentPosition() { return currentPositionIndex; }
    public int getCurrentPositionIndex() { return currentPositionIndex; }
    public int getMotorPosition() { return spinDexMotor.getCurrentPosition(); }
    public double getTargetPositionTicks() { return targetPositionTicks; }
    public int getCurrentTurn() { return currentPositionIndex / 6; }
    public double getServoPosition() { return targetPositionTicks; } // For telemetry compatibility

    // --- TUNING: set PD to whatever you want ---
    public void setPDGains(double Kp, double Kd) {
        this.kp = Kp;
        this.kd = Kd;
        applyBuiltInPD(this.kp, this.kd);
    }

    private void applyBuiltInPD(double Kp, double Kd) {
        // I=0, F=0 (you can add F later if you want)
        PIDFCoefficients pidf = new PIDFCoefficients(Kp, 0.0, Kd, 0.0);

        // Position loop
        spinDexMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);

        // RUN_TO_POSITION also uses encoder/velocity loop internally, so set this too for consistency. :contentReference[oaicite:2]{index=2}
        spinDexMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void resetEncoder() {
        spinDexMotor.setPower(0);
        spinDexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        currentPositionIndex = 0;
        targetPositionTicks = 0;

        spinDexMotor.setTargetPosition(0);
        spinDexMotor.setTargetPositionTolerance(POSITION_TOLERANCE_TICKS);
        spinDexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        applyBuiltInPD(kp, kd);
    }
}
