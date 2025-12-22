package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class ShooterMacroPGP {

    private final Telemetry telemetry;

    private enum MacroState {
        IDLE,                   // Not running
        ALIGNING,               // Moving SpinDex to the closest filled slot
        SPIN_UP,                // Waiting for shooter to reach target velocity
        PUSHING,                // Firing the pusher servo
        CLEANUP,                // Waiting for pusher to return and clearing slot
        COMPLETE,               // Macro finished
        FAILED_EMPTY            // Failed because no filled slots were found
    }

    private final SpinDex spinDex;
    private final Shooter shooter;
    private final Pusher pusher;
    private String [] motif = new String [] {"p", "g", "p"};
    int i = 0;

    private MacroState state = MacroState.IDLE;
    private long stateStartTime = 0;
    private double targetVelocity = HIGH_VELOCITY_THRESHOLD; // Default target
    private int currentSlotIndex = -1;                      // The logical slot index being fired (0, 1, or 2)

    // Configuration for the macro
    private static final double VELOCITY_TOLERANCE_TS = 20.0; // Tolerance for velocity error (ticks/s)

    public ShooterMacroPGP(SpinDex spinDex, Shooter shooter, Pusher pusher, Telemetry telemetry) {
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
    }

    // Start the shooting sequence
    public void start(double velocity) {
        if (spinDex.isEmpty()) {
            state = MacroState.FAILED_EMPTY;
            return;
        }

        // Set shooter to target velocity immediately
        this.targetVelocity = velocity;
        shooter.setVelocity(velocity);

        state = MacroState.ALIGNING;
        stateStartTime = System.currentTimeMillis();
        currentSlotIndex = -1; // Reset slot tracking
    }

    // Immediately stop the macro and all related components
    public void stop() {
        // Do NOT stop the shooter here, as the operator controls might be using it
        pusher.stop();
        state = MacroState.IDLE;
    }

    public void update() {
        if (state == MacroState.IDLE || state == MacroState.COMPLETE || state == MacroState.FAILED_EMPTY)
            return;

        long currentTime = System.currentTimeMillis();

        switch (state) {
            case ALIGNING:
                // 1. Move to the closest filled slot for shooting
                boolean found;
                if(motif[i].equals("g")) found = spinDex.moveToGreenArtifact();
                else found=spinDex.moveToPurpleArtifact();
                if (!found) {
                    // This check is mainly a safeguard if the state changed after start() was called
                    state = MacroState.FAILED_EMPTY;
                    break;
                }

                // Wait for the servo to travel
                if (currentTime - stateStartTime >= OUTTAKE_SERVO_TRAVEL_TIME_MS) {
                    // Determine the logical slot index we are now aligned to
                    int currentPos = spinDex.getCurrentPosition();
                    int posInTurn = currentPos % 6; // Get 0-5

                    // Reverse map the shooting position with new offset of 3:
                    // Shooting positions: 3->Slot0, 5->Slot1, 1->Slot2
                    if (posInTurn == 3) currentSlotIndex = 0;
                    else if (posInTurn == 5) currentSlotIndex = 1;
                    else if (posInTurn == 1) currentSlotIndex = 2;

                    state = MacroState.SPIN_UP;
                    stateStartTime = currentTime;
                }
                i++;
                i = i%3;
                break;

            case SPIN_UP:
                // 2. Wait for the shooter to reach the target velocity
                double velocityError = shooter.getVelocityError();

                // Check if the velocity error is within the acceptable tolerance
                if (Math.abs(velocityError) <= VELOCITY_TOLERANCE_TS) {
                    if (pusher.isReady()) {
                        // Shooter is ready, Pusher is ready, FIRE!
                        pusher.push();
                        state = MacroState.PUSHING;
                        stateStartTime = currentTime;
                    }
                }

                break;

            case PUSHING:
                // 3. Wait for the pusher to complete its sequence
                // The Pusher state machine handles the extend/retract cycle automatically
                // Use isReady() instead of string comparison for robustness
                if (pusher.isReady()) {
                    // The push is complete, the artifact is fired.
                    state = MacroState.CLEANUP;
                    stateStartTime = currentTime;
                }
                break;

            case CLEANUP:
                // 4. Update Spindex and determine next action
                if (currentSlotIndex != -1) {
                    spinDex.clearSlot(currentSlotIndex); // Mark the slot as empty
                } else {
                    telemetry.addData("MACRO WARNING", "Could not clear slot - invalid index");
                }

                // Give a moment for things to settle before declaring COMPLETE
                if (currentTime - stateStartTime >= MOVE_DELAY_MS) {
                    if (spinDex.isEmpty()) {
                        // If everything is clear, we are done
                        state = MacroState.COMPLETE;
                    } else {
                        // If more artifacts exist, loop back to align the next one
                        // This turns the macro into a continuous "fire all" sequence
                        state = MacroState.ALIGNING;
                        stateStartTime = currentTime;
                    }
                }
                break;
        }
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
        if (state == MacroState.SPIN_UP) {
            telemetry.addData("Vel Status", "Waiting... Error: %.0f t/s", shooter.getVelocityError());
        }
        if (state == MacroState.PUSHING || state == MacroState.CLEANUP) {
            telemetry.addData("Firing Slot", currentSlotIndex);
        }
    }
}