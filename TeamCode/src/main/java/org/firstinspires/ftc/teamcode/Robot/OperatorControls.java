package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;

import java.util.Locale;

public class OperatorControls {

    // Subsystems
    private final Intake intake;
    private final SpinDex spinDex;
    private final Shooter shooter;
    private final Pusher pusher;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;
    private final IntakeMacro intakeMacro;

    // ============================================================
    // INTAKE STATE MACHINE
    // ============================================================

    private enum IntakeState { OFF, INTAKING, SPITTING, MACRO_RUNNING }
    private IntakeState intakeState = IntakeState.OFF;

    // ============================================================
    // SHOOTER STATE MACHINE
    // ============================================================

    private enum ShooterMode { OFF, LOW_VELOCITY, HIGH_VELOCITY }
    private ShooterMode shooterMode = ShooterMode.OFF;
    private double shooterVelocity = 0.0; // Target velocity in ticks/sec

    // Velocity increment (5% of max velocity)
    private static final double VELOCITY_INCREMENT = SHOOTER_MAX_VELOCITY * 0.05;

    // Velocity thresholds for mode detection (from HamiltonParams)
    private static final double LOW_VELOCITY_THRESHOLD = HamiltonParams.LOW_VELOCITY_THRESHOLD;
    private static final double HIGH_VELOCITY_THRESHOLD = HamiltonParams.HIGH_VELOCITY_THRESHOLD;

    // ============================================================
    // BUTTON HELPERS — EDGE DETECTORS
    // ============================================================

    private final ButtonHelper btnCross = new ButtonHelper(); // Intake toggle
    private final ButtonHelper btnCircle = new ButtonHelper(); // Spit toggle
    private final ButtonHelper btnSquare = new ButtonHelper(); // Fire pusher
    private final ButtonHelper btnL1 = new ButtonHelper(); // Shooter power -
    private final ButtonHelper btnR1 = new ButtonHelper(); // Shooter power +
    private final ButtonHelper btnDpadUp = new ButtonHelper(); // Intake Macro
    private final ButtonHelper btnDpadDown = new ButtonHelper();
    private final ButtonHelper btnDpadLeft = new ButtonHelper(); // Spindex left
    private final ButtonHelper btnDpadRight = new ButtonHelper(); // Spindex right
    private final ButtonHelper btnShare = new ButtonHelper(); // E-Stop

    private static final int SPINDEX_MAX_POSITIONS = 6;

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public OperatorControls(Intake intake,
                            SpinDex spinDex,
                            Shooter shooter,
                            Pusher pusher,
                            Telemetry telemetry,
                            ColorSensor colorSensor) {

        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;

        // Initialize the intake macro
        this.intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, telemetry);
    }

    // ============================================================
    // MAIN UPDATE ENTRY POINT
    // ============================================================

    public void update(Gamepad g2) {
        handleIntake(g2);
        handleShooter(g2);
        handlePusher(g2);
        handleSpindex(g2);
        handleEmergencyStop(g2);

        // Update macro
        intakeMacro.update();

        // Check if macro completed - reset and allow restart
        if (intakeMacro.isComplete()) {
            intakeState = IntakeState.OFF;
            // Don't need to call reset() here since start() will handle it
        }

        pusher.update();
    }

    // ============================================================
    // INTAKE HANDLING
    // ============================================================

    private void handleIntake(Gamepad g2) {

        // Start Intake Macro (DPad Up)
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            if (intakeState != IntakeState.MACRO_RUNNING) {
                // Stop any manual intake
                intake.stop();

                // Start the macro
                intakeMacro.start();
                intakeState = IntakeState.MACRO_RUNNING;

                telemetry.addData("Status", "Intake Macro Started");
                telemetry.update();
            } else {
                // Stop the macro if already running
                intakeMacro.stop();
                intakeState = IntakeState.OFF;

                telemetry.addData("Status", "Intake Macro Stopped");
                telemetry.update();
            }
        }

        // Clear all slots (DPad Down)
        if (btnDpadDown.wasPressed(g2.dpad_down)) {
            spinDex.clearAllSlots();
            telemetry.addData("Status", "All slots cleared!");
            telemetry.update();
        }

        // Don't allow manual intake controls while macro is running
        if (intakeState == IntakeState.MACRO_RUNNING) {
            return;
        }

        // Toggle Intake (Cross / A)
        if (btnCross.wasPressed(g2.cross)) {

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

        // Toggle Spit (Circle / B)
        if (btnCircle.wasPressed(g2.circle)) {

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

    // ============================================================
    // SHOOTER HANDLING (VELOCITY-BASED WITH 5% INCREMENTS)
    // ============================================================

    private void handleShooter(Gamepad g2) {

        // Quick preset: R1 + Triangle = 80% velocity
        if (btnR1.wasPressed(g2.right_bumper) && g2.triangle) {
            shooterVelocity = SHOOTER_MAX_VELOCITY * 0.80;
        }
        // Quick preset: L1 + Triangle = 0% velocity (stop)
        else if (btnL1.wasPressed(g2.left_bumper) && g2.triangle) {
            shooterVelocity = 0.0;
        }
        // Normal increment: R1 alone = +5%
        else if (btnR1.wasPressed(g2.right_bumper)) {
            shooterVelocity = Math.min(SHOOTER_MAX_VELOCITY, shooterVelocity + VELOCITY_INCREMENT);
        }
        // Normal decrement: L1 alone = -5%
        else if (btnL1.wasPressed(g2.left_bumper)) {
            shooterVelocity = Math.max(0.0, shooterVelocity - VELOCITY_INCREMENT);
        }

        // Set the velocity
        shooter.setVelocity(shooterVelocity);

        // Mode assignment based on ACTUAL VELOCITY, not target
        double currentVelocity = shooter.getAverageVelocity();

        if (currentVelocity < LOW_VELOCITY_THRESHOLD) {
            shooterMode = ShooterMode.OFF;
        } else if (currentVelocity < HIGH_VELOCITY_THRESHOLD) {
            shooterMode = ShooterMode.LOW_VELOCITY;
        } else {
            shooterMode = ShooterMode.HIGH_VELOCITY;
        }
    }

    // ============================================================
    // PUSHER (FIRING) HANDLING
    // ============================================================

    private void handlePusher(Gamepad g2) {
        if (btnSquare.wasPressed(g2.square) && pusher.isReady()) {
            pusher.push();
        }
    }

    // ============================================================
    // SPINDEX HANDLING
    // ============================================================

    private void handleSpindex(Gamepad g2) {

        // Don't allow manual spindex control during macro
        if (intakeState == IntakeState.MACRO_RUNNING) {
            return;
        }

        // Dpad Right — Next position
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(next);
        }

        // Dpad Left — Previous position
        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            int prev = spinDex.getCurrentPosition() - (g2.triangle ? 1 : 2);

            // Wraparound
            if (prev < 0) prev += SPINDEX_MAX_POSITIONS;

            spinDex.moveToPosition(prev);
        }
    }

    // ============================================================
    // EMERGENCY STOP
    // ============================================================

    private void handleEmergencyStop(Gamepad g2) {

        if (btnShare.wasPressed(g2.share)) {

            stopAll();

            // Stop macro
            intakeMacro.stop();

            // Clear all slot tracking
            spinDex.clearAllSlots();

            intakeState = IntakeState.OFF;
            shooterMode = ShooterMode.OFF;

            // Reset helpers (force clean state)
            btnCross.reset();
            btnCircle.reset();
            btnSquare.reset();
            btnL1.reset();
            btnR1.reset();
            btnDpadLeft.reset();
            btnDpadRight.reset();
            btnDpadUp.reset();
            btnDpadDown.reset();

            telemetry.addData("!!! EMERGENCY STOP !!!", "ALL SYSTEMS HALTED");
            telemetry.update();
        }
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    public void updateTelemetry() {

        // Shooter
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Actual Vel", "%.0f t/s", shooter.getAverageVelocity());
        telemetry.addData("Vel Error", "%.0f t/s", shooter.getVelocityError());

        // Intake
        // telemetry.addData("Intake State", intake.getState());

        // Macro status
        if (intakeState == IntakeState.MACRO_RUNNING) {
            intakeMacro.addTelemetry();
        }

        // Pusher
        // telemetry.addData("Pusher State", pusher.getState());
        // telemetry.addData("Pusher Servo", "%.3f", pusher.getServoPosition());

        // Spindex
        telemetry.addData("Spindex Pos", "%d/%d", spinDex.getCurrentPosition(), SPINDEX_MAX_POSITIONS);
        telemetry.addData("Spindex Servo", "%.3f", spinDex.getServoPosition());

        telemetry.addLine("Slots:");
        telemetry.addData("0", spinDex.getSlot(0));
        telemetry.addData("1", spinDex.getSlot(1));
        telemetry.addData("2", spinDex.getSlot(2));

        // Color Sensors
        telemetry.addData("Color Left", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color Right", colorSensor.getDetailedColorInfoR());
    }

    // ============================================================
    // GLOBAL STOP
    // ============================================================

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        intakeMacro.stop();
    }
}