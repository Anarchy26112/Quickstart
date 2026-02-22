package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class IntakeMacro {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        FIND_AND_ALIGN, // New simplified state
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
    private static final int REQUIRED_DETECTIONS = 1;
    private boolean alignCommanded = false;


    private SpinDex.ArtifactType cachedArtifact = SpinDex.ArtifactType.EMPTY;

    public IntakeMacro(Intake intake, SpinDex spinDex, ColorSensor colorSensor, Shooter shooter, Telemetry telemetry) {
        this.intake = intake;
        this.spinDex = spinDex;
        this.colorSensor = colorSensor;
        this.shooter = shooter;
        this.telemetry = telemetry;
    }

    public void start() {
        if (spinDex.isFull()) { state = MacroState.COMPLETE; return; }

        state = MacroState.FIND_AND_ALIGN;
        consecutiveDetections = 0;
        cachedArtifact = SpinDex.ArtifactType.EMPTY;
        stateStartTime = System.currentTimeMillis();

        alignCommanded = false;
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
                    stateStartTime = currentTime; // optional: track when we commanded
                    state = MacroState.INTAKING;
                    intake.intake();
                }
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
                    // We can ask SpinDex which logical slot matches the current position
                    int currentPos = spinDex.getCurrentPosition();

                    // Logic map from SpinDex class: Slot 0->Pos 0, Slot 1->Pos 2, Slot 2->Pos 4
                    // We need to reverse map this to save the data correctly.
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
                }
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

            // Prioritize Left, fallback to Right
            ColorSensor.DetectedColor bestColor = (right != ColorSensor.DetectedColor.UNKNOWN) ? right : left;

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
        if (state == MacroState.INTAKING) {
            telemetry.addData("Detecting...", "%d/%d", consecutiveDetections, REQUIRED_DETECTIONS);
        }

    }
}