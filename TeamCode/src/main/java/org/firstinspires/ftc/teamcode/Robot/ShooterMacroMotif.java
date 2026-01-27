package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class ShooterMacroMotif {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        ALIGNING,        // Command SpinDex once, then wait until at target OR timeout
        SPIN_UP,         // Time-based style: wait for pusher ready (no velocity debounce)
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

    // Match ShooterMacro time-based align timeout
    private static final long ALIGN_TIMEOUT_MS = OUTTAKE_SERVO_TRAVEL_TIME_MS;

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

        // Default velocity if caller passed 0
        this.targetVelocity = (velocity > 0) ? velocity : HIGH_VELOCITY_THRESHOLD;
        shooter.setVelocity(this.targetVelocity);

        state = MacroState.ALIGNING;
        stateStartTime = System.currentTimeMillis();

        alignCommanded = false;
        currentSlotIndex = -1;
        motifIndex = 0;
    }

    public void stop() {
        pusher.stop();
        state = MacroState.IDLE;

        alignCommanded = false;
        currentSlotIndex = -1;
    }

    public void update() {
        if (state == MacroState.IDLE || state == MacroState.COMPLETE || state == MacroState.FAILED_EMPTY) return;

        long now = System.currentTimeMillis();

        switch (state) {

            case ALIGNING: {
                // Command move only ONCE per shot
                if (!alignCommanded) {
                    boolean found = commandMotifMoveOnce();
                    if (!found) {
                        state = MacroState.FAILED_EMPTY;
                        break;
                    }
                    alignCommanded = true;
                    stateStartTime = now;
                }

                boolean atTarget = spinDex.isAtTarget();
                boolean timedOut = (now - stateStartTime) >= ALIGN_TIMEOUT_MS;

                if (atTarget || timedOut) {
                    if (timedOut && !atTarget) {
                        telemetry.addData("MACRO WARNING", "Spindex ALIGN timeout; continuing");
                    }

                    // Determine slot index from current position (same mapping you used)
                    int posInTurn = Math.floorMod(spinDex.getCurrentPosition(), 6);

                    // Shooting positions w/ offset 3: 3->Slot0, 5->Slot1, 1->Slot2
                    if (posInTurn == 3) currentSlotIndex = 0;
                    else if (posInTurn == 5) currentSlotIndex = 1;
                    else if (posInTurn == 1) currentSlotIndex = 2;
                    else currentSlotIndex = -1;

                    state = MacroState.SPIN_UP;
                    stateStartTime = now;
                }
                break;
            }

            case SPIN_UP: {
                // Time-based style like ShooterMacro: don't debounce shooter speed here
                if (pusher.isReady()) {
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

                if (spinDex.isEmpty()) {
                    shooter.stop(); // optional, matches ShooterMacro style
                    state = MacroState.COMPLETE;
                } else {
                    // loop to fire next artifact
                    state = MacroState.ALIGNING;
                    stateStartTime = now;

                    alignCommanded = false;
                    currentSlotIndex = -1;

                    // keep shooter spinning at same targetVelocity
                    shooter.setVelocity(targetVelocity);
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
        else found = spinDex.moveToNextFilledSlotForShooting();

        motifIndex = (motifIndex + 1) % motif.length;
        return found;
    }

    public boolean isRunning() {
        return state != MacroState.IDLE && state != MacroState.COMPLETE && state != MacroState.FAILED_EMPTY;
    }

    public boolean isComplete() { return state == MacroState.COMPLETE; }
    public boolean hasFailed() { return state == MacroState.FAILED_EMPTY; }

    public void addTelemetry() {
        telemetry.addData("Shooter Macro Motif", state);
        telemetry.addData("Target Vel", "%.0f", targetVelocity);

        if (state == MacroState.ALIGNING) {
            telemetry.addData("Align Elapsed (ms)", "%d/%d",
                    (System.currentTimeMillis() - stateStartTime), ALIGN_TIMEOUT_MS);
            telemetry.addData("Spindex AtTarget", spinDex.isAtTarget());
            telemetry.addData("Spindex Pos", spinDex.getCurrentPosition());
        }

        if (state == MacroState.PUSHING || state == MacroState.CLEANUP) {
            telemetry.addData("Firing Slot", currentSlotIndex);
        }
    }
}
