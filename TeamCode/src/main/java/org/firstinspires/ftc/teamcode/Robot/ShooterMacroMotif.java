package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

/**
 * Motif-based shooter macro that behaves IDENTICALLY to ShooterMacro:
 *  - Commands SpinDex move ONLY ONCE per shot (alignCommanded)
 *  - Waits on spinDex.isAtTarget() (not time-based guess)
 *  - Debounces shooter "at speed" using REQUIRED_READY_CYCLES
 *  - Defaults velocity if caller passes 0
 *  - Clears the fired logical slot, then loops until empty
 *
 * Motif examples:
 *  - "gpp" = green, purple, purple
 *  - "gpg" = green, purple, green
 *  - "ppg" = purple, purple, green
 */
public class ShooterMacroMotif {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        ALIGNING,        // Command SpinDex once, then wait until at target
        SPIN_UP,         // Wait for shooter to be stably at target speed
        PUSHING,         // Wait for pusher cycle to finish
        CLEANUP,         // Clear slot, optionally continue
        COMPLETE,
        FAILED_EMPTY
    }

    private final SpinDex spinDex;
    private final Shooter shooter;
    private final Pusher pusher;

    private MacroState state = MacroState.IDLE;
    private long stateStartTime = 0;

    private double targetVelocity = HIGH_VELOCITY_THRESHOLD;

    private boolean alignCommanded = false;
    private int currentSlotIndex = -1;

    // Motif control
    private final String[] motif;   // array like {"g","p","p"}
    private int motifIndex = 0;

    // Shooter ready tuning (same as ShooterMacro)
    private static final double VELOCITY_TOLERANCE_TS = 20.0; // ticks/sec tolerance
    private static final int REQUIRED_READY_CYCLES = 2;       // debounce
    private int readyCycles = 0;

    public ShooterMacroMotif(SpinDex spinDex,
                             Shooter shooter,
                             Pusher pusher,
                             Telemetry telemetry,
                             String[] motif) {
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.motif = (motif == null || motif.length == 0) ? new String[]{"p"} : motif;
    }

    public void start(double velocity) {
        if (spinDex.isEmpty()) {
            state = MacroState.FAILED_EMPTY;
            return;
        }

        // If caller passed 0 (common), default so macro actually spins up
        this.targetVelocity = (velocity > 0) ? velocity : HIGH_VELOCITY_THRESHOLD;
        shooter.setVelocity(this.targetVelocity);

        state = MacroState.ALIGNING;
        stateStartTime = System.currentTimeMillis();

        alignCommanded = false;
        currentSlotIndex = -1;
        readyCycles = 0;
        motifIndex = 0;
    }

    public void stop() {
        pusher.stop();
        state = MacroState.IDLE;

        alignCommanded = false;
        currentSlotIndex = -1;
        readyCycles = 0;
    }

    public void update() {
        if (state == MacroState.IDLE || state == MacroState.COMPLETE || state == MacroState.FAILED_EMPTY) return;

        long now = System.currentTimeMillis();

        switch (state) {

            case ALIGNING: {
                // Command move only ONCE per shot (critical fix)
                if (!alignCommanded) {
                    boolean found = commandMotifMoveOnce();
                    if (!found) {
                        state = MacroState.FAILED_EMPTY;
                        break;
                    }
                    alignCommanded = true;
                    stateStartTime = now;
                }

                // Wait for motor to actually reach target (not a time guess)
                if (spinDex.isAtTarget()) {
                    int posInTurn = Math.floorMod(spinDex.getCurrentPosition(), 6);

                    // Shooting positions w/ offset 3: 3->Slot0, 5->Slot1, 1->Slot2
                    if (posInTurn == 3) currentSlotIndex = 0;
                    else if (posInTurn == 5) currentSlotIndex = 1;
                    else if (posInTurn == 1) currentSlotIndex = 2;
                    else currentSlotIndex = -1; // should not happen if at target, but safe

                    state = MacroState.SPIN_UP;
                    stateStartTime = now;
                    readyCycles = 0;
                }
                break;
            }

            case SPIN_UP: {
                double err = shooter.getVelocityError();
                boolean atSpeed = Math.abs(err) <= VELOCITY_TOLERANCE_TS;

                if (atSpeed) readyCycles++;
                else readyCycles = 0;

                // Require stability + pusher ready
                if (readyCycles >= REQUIRED_READY_CYCLES && pusher.isReady()) {
                    pusher.push();
                    state = MacroState.PUSHING;
                    stateStartTime = now;
                }
                break;
            }

            case PUSHING: {
                if (pusher.isReady()) {
                    state = MacroState.CLEANUP;
                    stateStartTime = now;
                }
                break;
            }

            case CLEANUP: {
                if (currentSlotIndex != -1) {
                    spinDex.clearSlot(currentSlotIndex);
                } else {
                    telemetry.addData("MACRO WARNING", "Invalid slot index; not cleared");
                }

                // Small settle delay (optional)
                if (now - stateStartTime >= MOVE_DELAY_MS) {
                    if (spinDex.isEmpty()) {
                        state = MacroState.COMPLETE;
                    } else {
                        // loop to fire next artifact
                        state = MacroState.ALIGNING;
                        stateStartTime = now;

                        alignCommanded = false;
                        currentSlotIndex = -1;
                        readyCycles = 0;

                        // keep shooter spinning at same targetVelocity
                        shooter.setVelocity(targetVelocity);
                    }
                }
                break;
            }
        }
    }

    /**
     * Commands exactly ONE SpinDex move based on motif[motifIndex],
     * then advances motifIndex for the NEXT shot.
     */
    private boolean commandMotifMoveOnce() {
        String m = motif[motifIndex];
        boolean found;

        if ("g".equalsIgnoreCase(m)) found = spinDex.moveToGreenArtifact();
        else if ("p".equalsIgnoreCase(m)) found = spinDex.moveToPurpleArtifact();
        else found = spinDex.moveToNextFilledSlotForShooting(); // fallback if motif char is weird

        motifIndex = (motifIndex + 1) % motif.length;
        return found;
    }

    public boolean isRunning() {
        return state != MacroState.IDLE && state != MacroState.COMPLETE && state != MacroState.FAILED_EMPTY;
    }

    public boolean isComplete() {
        return state == MacroState.COMPLETE;
    }

    public boolean hasFailed() {
        return state == MacroState.FAILED_EMPTY;
    }

    public void addTelemetry() {
        telemetry.addData("Shooter Macro", state);
        telemetry.addData("Target Vel", "%.0f", targetVelocity);

        if (state == MacroState.ALIGNING) {
            telemetry.addData("Spindex AtTarget", spinDex.isAtTarget());
            telemetry.addData("Spindex Pos", spinDex.getCurrentPosition());
        }

        if (state == MacroState.SPIN_UP) {
            telemetry.addData("Vel Err", "%.0f t/s", shooter.getVelocityError());
            telemetry.addData("Ready Cycles", "%d/%d", readyCycles, REQUIRED_READY_CYCLES);
        }

        if (state == MacroState.PUSHING || state == MacroState.CLEANUP) {
            telemetry.addData("Firing Slot", currentSlotIndex);
        }
    }
}
