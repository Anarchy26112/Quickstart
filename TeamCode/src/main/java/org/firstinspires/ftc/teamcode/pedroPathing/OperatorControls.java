package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OperatorControls {
    private final Intake intake;
    private final SpinDex spinDex;
    private final Shooter shooter;
    private final TelemetryManager telemetryM;

    private boolean intakeActive = false;
    private boolean intakeWasActiveBeforeSpit = false;
    private boolean lastAPressed = false;
    private boolean lastBPressed = false;
    private boolean lastDpadUpPressed = false;
    private boolean lastDpadRightPressed = false;
    private boolean lastDpadDownPressed = false;
    private boolean lastDpadLeftPressed = false;

    private static final double SHOOTER_HIGH_POWER = 1.0;
    private static final double SHOOTER_LOW_POWER = 0.6;
    private static final int SPINDEX_MAX_POSITIONS = 6;

    public OperatorControls(Intake intake, SpinDex spinDex, Shooter shooter, TelemetryManager telemetryM) {
        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.telemetryM = telemetryM;
    }

    public void update(Gamepad gamepad2) {
        // ========== INTAKE CONTROLS ==========
        if (gamepad2.b && !lastBPressed) {
            intakeWasActiveBeforeSpit = intakeActive;
            intake.spit();
            intakeActive = false;
        } else if (!gamepad2.b && lastBPressed) {
            if (intakeWasActiveBeforeSpit) {
                intake.intake();
                intakeActive = true;
            } else {
                intake.stop();
            }
        }
        lastBPressed = gamepad2.b;

        if (!gamepad2.b && gamepad2.a && !lastAPressed) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intake();
            } else {
                intake.stop();
            }
        }
        lastAPressed = gamepad2.a;

        // ========== SHOOTER CONTROLS ==========
        if (gamepad2.left_trigger > 0.1) {
            if (gamepad2.right_trigger > 0.1) {
                shooter.spin(gamepad2.right_trigger);
            } else {
                shooter.spin(SHOOTER_HIGH_POWER);
            }
        } else if (gamepad2.left_bumper) {
            shooter.spin(SHOOTER_LOW_POWER);
        } else if (gamepad2.right_bumper) {
            shooter.spin(SHOOTER_HIGH_POWER);
        } else {
            shooter.stop();
        }

        // ========== SPINDEX CONTROLS (OPTION 1: SEQUENTIAL) ==========

        // D-Pad Right - Next position (cycle forward: 1→2→3→4→5→6→1...)
        if (gamepad2.dpad_right && !lastDpadRightPressed) {
            int nextPos = (spinDex.getCurrentPosition() % SPINDEX_MAX_POSITIONS) + 1;
            spinDex.moveToPosition(nextPos);
        }
        lastDpadRightPressed = gamepad2.dpad_right;

        // D-Pad Left - Previous position (cycle backward: 6←5←4←3←2←1←6...)
        if (gamepad2.dpad_left && !lastDpadLeftPressed) {
            int prevPos = spinDex.getCurrentPosition() - 1;
            if (prevPos < 1) prevPos = SPINDEX_MAX_POSITIONS; // Wrap around to position 6
            spinDex.moveToPosition(prevPos);
        }
        lastDpadLeftPressed = gamepad2.dpad_left;

        // D-Pad Up - Quick jump to home/start position
        if (gamepad2.dpad_up && !lastDpadUpPressed) {
            spinDex.moveToPosition(1); // Home position
        }
        lastDpadUpPressed = gamepad2.dpad_up;

        // D-Pad Down - Quick jump to middle position
        if (gamepad2.dpad_down && !lastDpadDownPressed) {
            spinDex.moveToPosition(4); // Middle position (adjust if needed)
        }
        lastDpadDownPressed = gamepad2.dpad_down;

        // ========== EMERGENCY STOP ==========
        if (gamepad2.y) {
            intake.stop();
            shooter.stop();
            intakeActive = false;
            intakeWasActiveBeforeSpit = false; // Also clear the saved state
            telemetryM.debug("⚠️ EMERGENCY STOP ACTIVATED");
            telemetryM.update();
        }
    }

    public void updateTelemetry() {
        telemetryM.debug("--- INTAKE ---");
        telemetryM.debug("State: " + intake.getState());
        telemetryM.debug("Power: " + String.format("%.2f", intake.getCurrentPower()));
        telemetryM.debug("Running: " + (intake.isRunning() ? "Yes" : "No"));

        telemetryM.debug("--- SHOOTER ---");
        telemetryM.debug("State: " + shooter.getState());
        telemetryM.debug("Power: " + String.format("%.2f", shooter.getCurrentPower()));
        telemetryM.debug("Velocities: R=" + String.format("%.0f", shooter.getRightVelocity()) +
                " | L=" + String.format("%.0f", shooter.getLeftVelocity()));
        telemetryM.debug("Balanced: " + (shooter.isBalanced(10.0) ? "Yes" : "No"));

        telemetryM.debug("--- SPINDEX ---");
        telemetryM.debug(spinDex.getState());
        telemetryM.debug("Servo: " + String.format("%.3f", spinDex.getServoPosition()));
        telemetryM.debug("Filled: " + spinDex.getFilledCount() + "/" + SPINDEX_MAX_POSITIONS);
        telemetryM.debug("Current slot: " + spinDex.getCurrentSlotColor());

        if (spinDex.hasColorSensor()) {
            telemetryM.debug("Sensor: " + spinDex.getDetectedColor() +
                    " @ " + String.format("%.1f cm", spinDex.getDistance()));
            telemetryM.debug("Artifact present: " + (spinDex.isArtifactPresent() ? "Yes" : "No"));
        }

        telemetryM.debug("--- CONTROLS ---");
        telemetryM.debug("GP2: A=Intake | B=Spit | Y=EmergStop");
        telemetryM.debug("GP2: LT=Shooter | RT=VarPower");
        telemetryM.debug("GP2: LB=LowShot | RB=HighShot");
        telemetryM.debug("GP2: D-Right=Next | D-Left=Prev | D-Up=Home | D-Down=Mid");
    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
    }
}