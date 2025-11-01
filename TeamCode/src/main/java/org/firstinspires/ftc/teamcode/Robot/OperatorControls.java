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
        OFF, LOW_POWER, HIGH_POWER, DIFFERENTIAL
    }
    private ShooterMode shooterMode = ShooterMode.OFF;

    // Button helpers - cleaner than manual tracking!
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
    private final ButtonHelper leftStickButton = new ButtonHelper();
    private final ButtonHelper rightStickButton = new ButtonHelper();

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
        // Check for differential mode first (both stick buttons pressed)
        if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
            shooterMode = ShooterMode.DIFFERENTIAL;
            shooter.setLPower((1 + gamepad2.left_stick_y) / 2);
            shooter.setRPower((1 + gamepad2.right_stick_y) / 2);

            return; // Exit early to prevent other modes from interfering
        }

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
                shooter.spin(SHOOTER_LOW_POWER);
                break;
            case HIGH_POWER:
                shooter.spin(SHOOTER_HIGH_POWER);
                break;
            case DIFFERENTIAL:
                // Already handled above
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

            // Reset all button states after E-Stop ---
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
            telemetryM.update();
        }

    }

    // ========== TELEMETRY ==========

    public void updateTelemetry() {

        telemetryM.addData("Intake State", intake.getState());
        telemetryM.addData("Running", intake.isRunning());
        telemetryM.addData("Power", String.format(Locale.US, "%.2f", intake.getCurrentPower()));



        telemetryM.addData("Shooter State", shooter.getState());
        telemetryM.addData("Avg Power", String.format(Locale.US, "%.2f", shooter.getCurrentPower()));
        telemetryM.addData("Left Power", String.format(Locale.US, "%.2f", shooter.getCurrentLPower()));
        telemetryM.addData("Right Power", String.format(Locale.US, "%.2f", shooter.getCurrentRPower()));
        telemetryM.addData("Avg Velocity", String.format(Locale.US, "%.1f", shooter.getAverageVelocity()));



        telemetryM.addData("Pusher State", pusher.getState());
        telemetryM.addData("Servo Pos", String.format(Locale.US, "%.3f", pusher.getServoPosition()));
        telemetryM.addData("Ready", pusher.isReady());



        telemetryM.addData("SpinDex State", spinDex.getState());
        telemetryM.addData("Current Slot", spinDex.getCurrentPosition());
        telemetryM.addData("Current Turn", spinDex.getCurrentTurn());
        telemetryM.addData("Servo Pos", String.format(Locale.US, "%.3f", spinDex.getServoPosition()));
        telemetryM.addData("Detected Color", spinDex.getDetectedColor());
        telemetryM.addData("Distance (cm)", String.format(Locale.US, "%.2f", spinDex.getDistance()));
        telemetryM.addData("Artifact Present", spinDex.isArtifactPresent());
        telemetryM.addData("Filled Slots", spinDex.getFilledCount() + "/3");


        telemetryM.update();
    }


    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
    }
}