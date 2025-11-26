package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class IntakeMacro {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        DETERMINE_START_SLOT, // New state to find first empty slot
        INTAKE_SLOT_0,
        WAITING_SLOT_0,
        MOVING_TO_SLOT_1,
        INTAKE_SLOT_1,
        WAITING_SLOT_1,
        MOVING_TO_SLOT_2,
        INTAKE_SLOT_2,
        WAITING_SLOT_2,
        COMPLETE
    }

    private final Intake intake;
    private final SpinDex spinDex;
    private final ColorSensor colorSensor;

    private MacroState state = MacroState.IDLE;
    private long stateStartTime = 0;

    // Debouncing and state tracking
    private int consecutiveDetections = 0;
    private static final int REQUIRED_DETECTIONS = 2; // Lowered slightly for responsiveness

    // Fix for Bug #1: Cache the color detected during the check
    private SpinDex.ArtifactType cachedArtifact = SpinDex.ArtifactType.EMPTY;

    public IntakeMacro(Intake intake, SpinDex spinDex, ColorSensor colorSensor, Telemetry telemetry) {
        this.intake = intake;
        this.spinDex = spinDex;
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;
    }

    public void start() {
        // Allow restart from any state
        state = MacroState.DETERMINE_START_SLOT;
        consecutiveDetections = 0;
        cachedArtifact = SpinDex.ArtifactType.EMPTY;
        stateStartTime = System.currentTimeMillis();
    }

    public void stop() {
        intake.stop();
        state = MacroState.IDLE;
        consecutiveDetections = 0;
    }

    public void update() {
        if (state == MacroState.IDLE || state == MacroState.COMPLETE) return;

        long currentTime = System.currentTimeMillis();

        switch (state) {
            case DETERMINE_START_SLOT:
                // Smart start: find the first empty slot
                int currentPos = spinDex.getCurrentPosition();
                int emptySlotIndex = -1;

                // Find first empty slot
                if (spinDex.getSlot(0) == SpinDex.ArtifactType.EMPTY) {
                    emptySlotIndex = 0;
                } else if (spinDex.getSlot(1) == SpinDex.ArtifactType.EMPTY) {
                    emptySlotIndex = 1;
                } else if (spinDex.getSlot(2) == SpinDex.ArtifactType.EMPTY) {
                    emptySlotIndex = 2;
                }

                // Check if all slots are full
                if (emptySlotIndex == -1) {
                    telemetry.addData("Macro Error", "All slots full!");
                    telemetry.update();
                    state = MacroState.COMPLETE;
                    return;
                }

                // Move to the appropriate loading position and transition to intake state
                spinDex.moveToSlot(emptySlotIndex, false);

                // Set the next state based on which slot we're filling
                switch (emptySlotIndex) {
                    case 0:
                        state = MacroState.INTAKE_SLOT_0;
                        break;
                    case 1:
                        state = MacroState.INTAKE_SLOT_1;
                        break;
                    case 2:
                        state = MacroState.INTAKE_SLOT_2;
                        break;
                }

                intake.intake();
                stateStartTime = currentTime;
                break;

            // ================= SLOT 0 =================
            case INTAKE_SLOT_0:
                // Start intake if we just arrived from DETERMINE_START_SLOT
                if (!intake.isRunning()) intake.intake();

                if (checkForBallAndCache()) {
                    spinDex.setSlot(0, cachedArtifact);
                    state = MacroState.WAITING_SLOT_0;
                    stateStartTime = currentTime;
                    consecutiveDetections = 0;
                }
                break;

            case WAITING_SLOT_0:
                if (currentTime - stateStartTime >= MOVE_DELAY_MS) {
                    state = MacroState.MOVING_TO_SLOT_1;
                    spinDex.moveToSlot(1, false);

                    cachedArtifact = SpinDex.ArtifactType.EMPTY; // Reset cache
                    stateStartTime = currentTime;
                }
                break;

            case MOVING_TO_SLOT_1:
                if (currentTime - stateStartTime >= SERVO_TRAVEL_TIME_MS) {
                    // Check if Slot 1 is already full (from previous run)
                    if (spinDex.getSlot(1) != SpinDex.ArtifactType.EMPTY) {
                        // Skip to moving to slot 2
                        state = MacroState.MOVING_TO_SLOT_2;
                        spinDex.moveToSlot(2, false);
                        stateStartTime = currentTime; // Reset timer for the next move
                    } else {
                        state = MacroState.INTAKE_SLOT_1;
                        intake.intake(); // Restart intake
                        stateStartTime = currentTime;
                    }
                }
                break;

            // ================= SLOT 1 =================
            case INTAKE_SLOT_1:
                // Intake is already running from previous state
                if (!intake.isRunning()) intake.intake(); // Safety check

                if (checkForBallAndCache()) {
                    spinDex.setSlot(1, cachedArtifact);
                    state = MacroState.WAITING_SLOT_1;
                    stateStartTime = currentTime;
                    consecutiveDetections = 0;
                }
                break;

            case WAITING_SLOT_1:
                if (currentTime - stateStartTime >= MOVE_DELAY_MS) {
                    state = MacroState.MOVING_TO_SLOT_2;
                    spinDex.moveToSlot(2, false);
                    cachedArtifact = SpinDex.ArtifactType.EMPTY; // Reset cache
                    stateStartTime = currentTime;
                }
                break;

            case MOVING_TO_SLOT_2:
                if (currentTime - stateStartTime >= SERVO_TRAVEL_TIME_MS) {
                    if (spinDex.getSlot(2) != SpinDex.ArtifactType.EMPTY) {
                        state = MacroState.COMPLETE;
                        intake.stop();
                    } else {
                        state = MacroState.INTAKE_SLOT_2;
                        intake.intake();
                        stateStartTime = currentTime;
                    }
                }
                break;

            // ================= SLOT 2 =================
            case INTAKE_SLOT_2:
                // Intake is already running from previous state
                if (!intake.isRunning()) intake.intake(); // Safety check

                if (checkForBallAndCache()) {
                    spinDex.setSlot(2, cachedArtifact);
                    state = MacroState.WAITING_SLOT_2;
                    stateStartTime = currentTime;
                    consecutiveDetections = 0;
                }
                break;

            case WAITING_SLOT_2:
                if (currentTime - stateStartTime >= MOVE_DELAY_MS) {
                    intake.stop();
                    state = MacroState.COMPLETE;
                }
                break;
        }
    }

    /**
     * Checks for ball and CACHES the color if found.
     * This fixes the issue where we detect a ball, but then re-read
     * the sensor and get UNKNOWN.
     */
    private boolean checkForBallAndCache() {
        ColorSensor.DetectedColor left = colorSensor.detectColorL();
        ColorSensor.DetectedColor right = colorSensor.detectColorR();

        boolean detected = (left != ColorSensor.DetectedColor.UNKNOWN) ||
                (right != ColorSensor.DetectedColor.UNKNOWN);

        if (detected) {
            consecutiveDetections++;

            // Determine tentative color for this frame
            SpinDex.ArtifactType frameColor = SpinDex.ArtifactType.EMPTY;

            // Prioritize Left, fallback to Right
            ColorSensor.DetectedColor bestColor = (left != ColorSensor.DetectedColor.UNKNOWN) ? left : right;

            if (bestColor == ColorSensor.DetectedColor.GREEN) frameColor = SpinDex.ArtifactType.GREEN;
            else if (bestColor == ColorSensor.DetectedColor.PURPLE) frameColor = SpinDex.ArtifactType.PURPLE;

            // Update the cache with the most recent valid color seen
            if (frameColor != SpinDex.ArtifactType.EMPTY) {
                cachedArtifact = frameColor;
            }

            return consecutiveDetections >= REQUIRED_DETECTIONS;
        } else {
            consecutiveDetections = 0;
            return false;
        }
    }

    public boolean isRunning() {
        return state != MacroState.IDLE && state != MacroState.COMPLETE;
    }

    public boolean isComplete() {
        return state == MacroState.COMPLETE;
    }

    public String getStateString() {
        return state.toString();
    }

    public void reset() {
        state = MacroState.IDLE;
        consecutiveDetections = 0;
    }

    public void addTelemetry() {
        telemetry.addData("Intake Macro", getStateString());
        if (isRunning()) {
            telemetry.addData("Detections", "%d/%d", consecutiveDetections, REQUIRED_DETECTIONS);
            telemetry.addData("Cached Color", cachedArtifact);
        }
    }
}