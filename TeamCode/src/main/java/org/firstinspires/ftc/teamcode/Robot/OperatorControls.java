package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

import java.util.Locale;

public class OperatorControls {
    private final Intake intake;
    private final SpinDex spinDex;
    private final Shooter shooter;
    private final Pusher pusher;
    private final TelemetryManager telemetryM;

    // Intake state management
    private enum IntakeState {
        OFF, INTAKING, SPITTING
    }
    private IntakeState intakeState = IntakeState.OFF;

    // Shooter state management
    private enum ShooterMode {
        OFF, LOW_POWER, HIGH_POWER
    }
    private ShooterMode shooterMode = ShooterMode.OFF;

    // Button helpers
    private final ButtonHelper aButton = new ButtonHelper();
    private final ButtonHelper bButton = new ButtonHelper();
    private final ButtonHelper xButton = new ButtonHelper();
    private final ButtonHelper leftBumper = new ButtonHelper();
    private final ButtonHelper rightBumper = new ButtonHelper();
    private final ButtonHelper dpadUp = new ButtonHelper();
    private final ButtonHelper dpadRight = new ButtonHelper();
    private final ButtonHelper dpadDown = new ButtonHelper();
    private final ButtonHelper dpadLeft = new ButtonHelper();
    private final ButtonHelper yButton = new ButtonHelper();

    private static final int SPINDEX_MAX_POSITIONS = 6;

    public OperatorControls(Intake intake, SpinDex spinDex, Shooter shooter, Pusher pusher, TelemetryManager telemetryM) {
        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetryM = telemetryM;
    }

    public void update(Gamepad gamepad2) {
        updateIntake(gamepad2);
        updateShooter(gamepad2);
        updatePusher(gamepad2);
        updateSpinDex(gamepad2);
        updateEmergencyStop(gamepad2);
    }

    // ========== INTAKE CONTROLS ==========

    private void updateIntake(Gamepad gamepad2) {
        // A button - Toggle intake
        if (aButton.wasPressed(gamepad2.a)) {
            switch (intakeState) {
                case INTAKING:
                    intake.stop();
                    intakeState = IntakeState.OFF;
                    break;
                default:
                    intake.intake();
                    intakeState = IntakeState.INTAKING;
                    break;
            }
        }

        // B button - Toggle spit
        if (bButton.wasPressed(gamepad2.b)) {
            switch (intakeState) {
                case SPITTING:
                    intake.stop();
                    intakeState = IntakeState.OFF;
                    break;
                default:
                    intake.spit();
                    intakeState = IntakeState.SPITTING;
                    break;
            }
        }
    }

    // ========== SHOOTER CONTROLS ==========

    private void updateShooter(Gamepad gamepad2) {
        // Left Bumper - Toggle low power mode
        if (leftBumper.wasPressed(gamepad2.left_bumper)) {
            if (shooterMode == ShooterMode.LOW_POWER) {
                shooterMode = ShooterMode.OFF;
            } else {
                shooterMode = ShooterMode.LOW_POWER;
            }
        }

        // Right Bumper - Toggle high power mode
        if (rightBumper.wasPressed(gamepad2.right_bumper)) {
            if (shooterMode == ShooterMode.HIGH_POWER) {
                shooterMode = ShooterMode.OFF;
            } else {
                shooterMode = ShooterMode.HIGH_POWER;
            }
        }

        // Apply shooter mode
        switch (shooterMode) {
            case LOW_POWER:
                shooter.setPower(SHOOTER_LOW_POWER);
                break;
            case HIGH_POWER:
                shooter.setPower(SHOOTER_HIGH_POWER);
                break;
            case OFF:
                shooter.stop();
                break;
        }
    }

    // ========== PUSHER CONTROLS ==========

    private void updatePusher(Gamepad gamepad2) {
        // X button - Push sample
        if (xButton.wasPressed(gamepad2.x) && pusher.isReady()) {
            pusher.push();
        }

        // Always update pusher state machine
        pusher.update();
    }

    // ========== SPINDEX CONTROLS ==========

    private void updateSpinDex(Gamepad gamepad2) {
        // D-pad Right - Next position
        if (dpadRight.wasPressed(gamepad2.dpad_right)) {
            int nextPos = (spinDex.getCurrentPosition() % SPINDEX_MAX_POSITIONS) + 1;
            spinDex.moveToPosition(nextPos);
        }

        // D-pad Left - Previous position
        if (dpadLeft.wasPressed(gamepad2.dpad_left)) {
            int prevPos = spinDex.getCurrentPosition() - 1;
            if (prevPos < 1) prevPos = SPINDEX_MAX_POSITIONS;
            spinDex.moveToPosition(prevPos);
        }

        // D-pad Up - Home position (1)
        if (dpadUp.wasPressed(gamepad2.dpad_up)) {
            spinDex.moveToPosition(1);
        }

        // D-pad Down - Middle position (4)
        if (dpadDown.wasPressed(gamepad2.dpad_down)) {
            spinDex.moveToPosition(4);
        }
    }

    // ========== EMERGENCY STOP ==========

    private void updateEmergencyStop(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.8 && gamepad2.left_trigger > 0.8 && yButton.wasPressed(gamepad2.y)) {
            stopAll();
            intakeState = IntakeState.OFF;
            shooterMode = ShooterMode.OFF;

            // Reset all button states after E-Stop
            aButton.reset();
            bButton.reset();
            xButton.reset();
            leftBumper.reset();
            rightBumper.reset();
            dpadUp.reset();
            dpadRight.reset();
            dpadDown.reset();
            dpadLeft.reset();

            telemetryM.debug("⚠️ EMERGENCY STOP ACTIVATED");
        }
    }

    // ========== TELEMETRY ==========

    public void updateTelemetry() {
        // Use debug() to add all telemetry lines
        telemetryM.debug(
                "🎯 SHOOTER",
                "  Mode: " + shooterMode.toString(),
                "  State: " + shooter.getState(),
                "  Avg Power: " + String.format(Locale.US, "%.2f", shooter.getCurrentPower()),
                "  Left Power: " + String.format(Locale.US, "%.2f", shooter.getCurrentLPower()),
                "  Right Power: " + String.format(Locale.US, "%.2f", shooter.getCurrentRPower()),
                "  Avg Velocity: " + String.format(Locale.US, "%.1f ticks/s", shooter.getAverageVelocity())
        );
        /*



        "",

                "════════════════════════════",
                "🎮 OPERATOR CONTROLS",
                "════════════════════════════",
                "",
                "🔄 INTAKE",
                "  State: " + intake.getState(),
                "  Running: " + intake.isRunning(),
                "  Power: " + String.format(Locale.US, "%.2f", intake.getCurrentPower()),
                "",


                "⚡ PUSHER",
                "  State: " + pusher.getState(),
                "  Ready: " + pusher.isReady(),
                "  Servo Pos: " + String.format(Locale.US, "%.3f", pusher.getServoPosition()),
                "",
                "🔀 SPINDEX",
                "  State: " + spinDex.getState(),
                "  Position: " + String.format(Locale.US, "%d/%d", spinDex.getCurrentPosition(), SPINDEX_MAX_POSITIONS),
                "  Turn: " + spinDex.getCurrentTurn(),
                "  Servo Pos: " + String.format(Locale.US, "%.3f", spinDex.getServoPosition()),
                "  Filled Slots: " + spinDex.getFilledCount() + "/3"



        */
        // Add color sensor data if available
        if (spinDex.hasColorSensor()) {
            telemetryM.debug(
                    "  Detected Color: " + spinDex.getDetectedColor(),
                    "  Distance: " + String.format(Locale.US, "%.2f cm", spinDex.getDistance()),
                    "  Artifact Present: " + spinDex.isArtifactPresent()
            );
        }

        telemetryM.debug(
                "",
                "🎮 CONTROLS",
                "  A/B: Intake/Spit",
                "  X: Push Sample",
                "  LB/RB: Shooter Mode",
                "  D-Pad: SpinDex Position",
                "  LT+RT+Y: Emergency Stop",
                "════════════════════════════"
        );

        // Update the telemetry display
        // Use update() for dashboard only, or update(telemetry) for both dashboard and Driver Station
        telemetryM.update();
    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
    }
}