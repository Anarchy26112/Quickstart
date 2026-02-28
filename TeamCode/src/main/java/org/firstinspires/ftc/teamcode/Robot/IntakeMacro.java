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
    private static final int REQUIRED_DETECTIONS = 1;
    private boolean alignCommanded = false;

    // Cache for detected artifact color
    private SpinDex.ArtifactType cachedArtifact = SpinDex.ArtifactType.EMPTY;

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

        // IMPORTANT: do not intake while moving
        intake.stop();
    }

    public void stop() {
        intake.stop();
        state = MacroState.IDLE;
        consecutiveDetections = 0;
        alignCommanded = false;
        cachedArtifact = SpinDex.ArtifactType.EMPTY;
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

                    // Do NOT intake until spindex reaches target
                    intake.stop();
                }

                // Wait here until SpinDex is at target
                if (!spinDex.isAtTarget()) {
                    // keep intake off while moving
                    if (intake.isRunning()) intake.stop();
                    break;
                }

                // Arrived: start intake and proceed
                if (!intake.isRunning()) intake.intake();
                state = MacroState.INTAKING;
                stateStartTime = currentTime;
                break;

            case INTAKING:
                // Ensure intake is running
                if (!intake.isRunning()) intake.intake();

                // Safety: if somehow we're not at target, don't trust sensors yet
                if (!spinDex.isAtTarget()) {
                    return;
                }

                // Check for ball entry
                if (checkForBallAndCache()) {
                    int currentPos = spinDex.getCurrentPosition();

                    // Slot 0->Pos 0, Slot 1->Pos 2, Slot 2->Pos 4 (mod 6)
                    int currentSlotIndex = -1;
                    int posInTurn = ((currentPos % 6) + 6) % 6;

                    if (posInTurn == 0) currentSlotIndex = 0;
                    else if (posInTurn == 2) currentSlotIndex = 1;
                    else if (posInTurn == 4) currentSlotIndex = 2;

                    if (currentSlotIndex != -1) {
                        spinDex.setSlot(currentSlotIndex, cachedArtifact);
                    }

                    state = MacroState.WAITING_FOR_SETTLE;
                    stateStartTime = currentTime;
                    consecutiveDetections = 0;

                    // Optional: stop intake while we decide next slot (keeps things clean)
                    intake.stop();
                }
                break;

            case WAITING_FOR_SETTLE:
                cachedArtifact = SpinDex.ArtifactType.EMPTY;

                if (spinDex.isFull()) {
                    state = MacroState.COMPLETE;
                    intake.stop();
                } else {
                    state = MacroState.FIND_AND_ALIGN;
                    alignCommanded = false;

                    // Make sure intake is off while we move to the next slot
                    intake.stop();
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
            telemetry.addData("Align Commanded", alignCommanded);
        }

        if (state == MacroState.INTAKING) {
            telemetry.addData("Detecting...", "%d/%d", consecutiveDetections, REQUIRED_DETECTIONS);
        }
    }
}