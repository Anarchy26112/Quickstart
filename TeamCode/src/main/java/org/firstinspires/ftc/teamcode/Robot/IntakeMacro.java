package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class IntakeMacro {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        FIND_AND_ALIGN,
        INTAKING,
        WAITING_FOR_SETTLE,
        COMPLETE
    }

    private final Intake intake;
    private final Shooter shooter;
    private final SpinDex spinDex;
    private final ColorSensor colorSensor;

    private MacroState state = MacroState.IDLE;
    private long stateStartTime = 0;

    // Debouncing and state tracking
    private int consecutiveDetections = 0;
    private static final int REQUIRED_DETECTIONS = 3;
    private boolean alignCommanded = false;

    // Cache for detected artifact color
    private SpinDex.ArtifactType cachedArtifact = SpinDex.ArtifactType.EMPTY;

    // One-time intake pause during SpinDex movement (per move command)
    private static final long ONE_STOP_MS = 300;
    private boolean oneStopApplied = false;
    private boolean oneStopInProgress = false;
    private long oneStopStartTime = 0;

    public IntakeMacro(Intake intake, SpinDex spinDex, ColorSensor colorSensor, Shooter shooter, Telemetry telemetry) {
        this.intake = intake;
        this.spinDex = spinDex;
        this.colorSensor = colorSensor;
        this.shooter = shooter;
        this.telemetry = telemetry;
    }

    public void start() {
        if (spinDex.isFull()) {
            state = MacroState.COMPLETE;
            return;
        }

        state = MacroState.FIND_AND_ALIGN;
        consecutiveDetections = 0;
        cachedArtifact = SpinDex.ArtifactType.EMPTY;
        stateStartTime = System.currentTimeMillis();

        alignCommanded = false;

        // reset one-time stop flags
        oneStopApplied = false;
        oneStopInProgress = false;
        oneStopStartTime = 0;
    }

    public void stop() {
        intake.stop();
        state = MacroState.IDLE;
        consecutiveDetections = 0;

        // reset one-time stop flags
        oneStopApplied = false;
        oneStopInProgress = false;
        oneStopStartTime = 0;
    }

    public void update() {
        if (state == MacroState.IDLE || state == MacroState.COMPLETE) return;

        long currentTime = System.currentTimeMillis();

        switch (state) {
            case FIND_AND_ALIGN:
                if (spinDex.isFull()) {
                    state = MacroState.COMPLETE;
                    intake.stop();
                    break;
                }

                if (!alignCommanded) {
                    boolean foundEmpty = spinDex.moveToNextEmptySlotForLoading();
                    if (!foundEmpty) {
                        state = MacroState.COMPLETE;
                        intake.stop();
                        break;
                    }
                    alignCommanded = true;

                    // Start intake immediately
                    intake.intake();

                    // Reset one-time stop flags for THIS move
                    oneStopApplied = false;
                    oneStopInProgress = false;
                    oneStopStartTime = 0;
                }

                // While SpinDex is moving, apply ONE 150ms stop exactly once
                if (!spinDex.isAtTarget()) {
                    if (!oneStopApplied) {
                        if (!oneStopInProgress) {
                            // begin the one-time stop
                            intake.stop();
                            oneStopInProgress = true;
                            oneStopStartTime = currentTime;
                        } else {
                            // end the stop after 150ms
                            if (currentTime - oneStopStartTime >= ONE_STOP_MS) {
                                intake.intake();
                                oneStopInProgress = false;
                                oneStopApplied = true; // ensures only once per move
                            }
                        }
                    } else {
                        // after the one stop is done, keep intake running while still moving
                        if (!intake.isRunning() && !oneStopInProgress) intake.intake();
                    }
                    break;
                }

                // Arrived at target: ensure intake is running and move to INTAKING
                if (!intake.isRunning()) intake.intake();
                state = MacroState.INTAKING;
                stateStartTime = currentTime;
                break;

            case INTAKING:
                // Ensure intake is running
                if (!intake.isRunning()) intake.intake();

                // Wait for travel time before checking for balls (prevent false positives during servo move)
                if (!spinDex.isAtTarget()) {
                    return;
                }

                // Check for ball entry
                if (checkForBallAndCache()) {
                    // We found a ball!

                    // 1. Figure out which slot we are currently filling
                    int currentPos = spinDex.getCurrentPosition();

                    // Logic map from SpinDex class: Slot 0->Pos 0, Slot 1->Pos 2, Slot 2->Pos 4
                    // Reverse map to save the data correctly.
                    int currentSlotIndex = -1;

                    int posInTurn = ((currentPos % 6) + 6) % 6; // Get 0-5
                    if (posInTurn == 0) currentSlotIndex = 0;
                    else if (posInTurn == 2) currentSlotIndex = 1;
                    else if (posInTurn == 4) currentSlotIndex = 2;

                    // 2. Save the artifact data
                    if (currentSlotIndex != -1) {
                        spinDex.setSlot(currentSlotIndex, cachedArtifact);
                    }

                    // 3. Move to wait state
                    state = MacroState.WAITING_FOR_SETTLE;
                    stateStartTime = currentTime;
                    consecutiveDetections = 0;
                }
                break;

            case WAITING_FOR_SETTLE:
                // Reset cache
                cachedArtifact = SpinDex.ArtifactType.EMPTY;

                // Check if we are full now
                if (spinDex.isFull()) {
                    state = MacroState.COMPLETE;
                    intake.stop();
                } else {
                    // Loop back to find the NEXT empty slot
                    state = MacroState.FIND_AND_ALIGN;
                    alignCommanded = false;

                    // IMPORTANT: reset one-time stop flags for the next move
                    oneStopApplied = false;
                    oneStopInProgress = false;
                    oneStopStartTime = 0;
                }
                break;

            case COMPLETE:
            case IDLE:
            default:
                break;
        }
    }

    private boolean checkForBallAndCache() {
        ColorSensor.DetectedColor left = colorSensor.detectColorL();
        ColorSensor.DetectedColor right = colorSensor.detectColorR();

        boolean detected = (left != ColorSensor.DetectedColor.UNKNOWN) ||
                (right != ColorSensor.DetectedColor.UNKNOWN);

        if (detected) {
            consecutiveDetections++;

            ColorSensor.DetectedColor bestColor = ColorSensor.DetectedColor.UNKNOWN;
            if (left != ColorSensor.DetectedColor.UNKNOWN) bestColor = left;
            else if (right != ColorSensor.DetectedColor.UNKNOWN) bestColor = right;

            SpinDex.ArtifactType frameColor = SpinDex.ArtifactType.EMPTY;
            if (bestColor == ColorSensor.DetectedColor.GREEN) frameColor = SpinDex.ArtifactType.GREEN;
            else if (bestColor == ColorSensor.DetectedColor.PURPLE) frameColor = SpinDex.ArtifactType.PURPLE;

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

    public void addTelemetry() {
        telemetry.addData("Intake Macro", state);

        if (state == MacroState.FIND_AND_ALIGN) {
            telemetry.addData("SpinDex At Target", spinDex.isAtTarget());
            telemetry.addData("OneStop Applied", oneStopApplied);
            telemetry.addData("OneStop InProgress", oneStopInProgress);
        }

        if (state == MacroState.INTAKING) {
            telemetry.addData("Detecting...", "%d/%d", consecutiveDetections, REQUIRED_DETECTIONS);
        }
    }
}