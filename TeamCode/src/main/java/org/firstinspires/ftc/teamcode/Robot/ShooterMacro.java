package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class ShooterMacro {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,
        ALIGNING,
        SPIN_UP,
        PUSHING,
        CLEANUP,
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

    // Shooter ready tuning
    private static final double VELOCITY_TOLERANCE_TS = 10.0;
    private static final int REQUIRED_READY_CYCLES = 6; // recommend >0 to avoid false-ready
    private int readyCycles = 0;

    private static final long ALIGN_TIMEOUT_MS = OUTTAKE_SERVO_TRAVEL_TIME_MS;

    public ShooterMacro(SpinDex spinDex, Shooter shooter, Pusher pusher, Telemetry telemetry) {
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
    }

    public void start(double velocity) {
        if (spinDex.isEmpty()) {
            state = MacroState.FAILED_EMPTY;
            return;
        }

        targetVelocity = (velocity > 0) ? velocity : HIGH_VELOCITY_THRESHOLD;
        shooter.setVelocity(targetVelocity);

        state = MacroState.ALIGNING;
        stateStartTime = System.currentTimeMillis();

        alignCommanded = false;
        currentSlotIndex = -1;
        readyCycles = 0;
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
                if (!alignCommanded) {
                    // Choose the best filled slot AND command the movement
                    int chosen = spinDex.getClosestFilledSlotIndexForShooting(); // <-- you add this helper (below)
                    if (chosen == -1) {
                        state = MacroState.FAILED_EMPTY;
                        break;
                    }

                    boolean moved = spinDex.moveToSpecificSlotForShooting(chosen); // <-- you add this helper (below)
                    if (!moved) {
                        state = MacroState.FAILED_EMPTY;
                        break;
                    }

                    currentSlotIndex = chosen;
                    alignCommanded = true;
                    stateStartTime = now;
                }

                boolean atTarget = spinDex.isAtTarget();
                boolean timedOut = (now - stateStartTime) >= ALIGN_TIMEOUT_MS;

                if (atTarget || timedOut) {
                    if (timedOut && !atTarget) {
                        telemetry.addData("MACRO WARNING", "Spindex ALIGN timeout; continuing");
                    }
                    state = MacroState.SPIN_UP;
                    stateStartTime = now;
                    readyCycles = 0;
                }
                break;
            }

            case SPIN_UP: {
                if (pusher.isReady()) {
                    pusher.push();
                    state = MacroState.PUSHING;
                    stateStartTime = now;
                }
                break;
            }

            case PUSHING: {
                // Wait for pusher to finish its cycle
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
                    shooter.stop();   // optional, but usually good
                    state = MacroState.COMPLETE;
                } else {
                    // loop to next shot
                    state = MacroState.ALIGNING;
                    stateStartTime = now;

                    alignCommanded = false;
                    currentSlotIndex = -1;
                    readyCycles = 0;

                    shooter.setVelocity(targetVelocity);
                }
                break;
            }
        }
    }

    public boolean isRunning() {
        return state != MacroState.IDLE && state != MacroState.COMPLETE && state != MacroState.FAILED_EMPTY;
    }

    public boolean isComplete() { return state == MacroState.COMPLETE; }
    public boolean hasFailed() { return state == MacroState.FAILED_EMPTY; }

    public void addTelemetry() {
        telemetry.addData("Shooter Macro", state);
        telemetry.addData("Target Vel", "%.0f", targetVelocity);

        if (state == MacroState.ALIGNING) {
            telemetry.addData("Align Elapsed (ms)", "%d/%d",
                    (System.currentTimeMillis() - stateStartTime), ALIGN_TIMEOUT_MS);
            telemetry.addData("Spindex AtTarget", spinDex.isAtTarget());
            telemetry.addData("Firing Slot (cmd)", currentSlotIndex);
        }

        if (state == MacroState.PUSHING || state == MacroState.CLEANUP) {
            telemetry.addData("Firing Slot", currentSlotIndex);
        }
    }
}
