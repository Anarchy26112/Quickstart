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

    // ============================================================
    // INTAKE STATE MACHINE
    // ============================================================

    private enum IntakeState { OFF, INTAKING, SPITTING }
    private IntakeState intakeState = IntakeState.OFF;

    // ============================================================
    // SHOOTER STATE MACHINE
    // ============================================================

    private enum ShooterMode { OFF, LOW_POWER, HIGH_POWER }
    private ShooterMode shooterMode = ShooterMode.OFF;
    private double shooterPower = 0.0;

    // ============================================================
    // BUTTON HELPERS — EDGE DETECTORS
    // ============================================================

    private final ButtonHelper btnCross = new ButtonHelper(); // Intake toggle
    private final ButtonHelper btnCircle = new ButtonHelper(); // Spit toggle
    private final ButtonHelper btnSquare = new ButtonHelper(); // Fire pusher
    private final ButtonHelper btnL1 = new ButtonHelper(); // Shooter power -
    private final ButtonHelper btnR1 = new ButtonHelper(); // Shooter power +
    private final ButtonHelper btnDpadUp = new ButtonHelper();
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

        pusher.update();
    }

    // ============================================================
    // INTAKE HANDLING
    // ============================================================

    private void handleIntake(Gamepad g2) {

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
    // SHOOTER HANDLING
    // ============================================================

    private void handleShooter(Gamepad g2) {

        // Increase power (R1)
        if (btnR1.wasPressed(g2.right_bumper)) {
            shooterPower = Math.min(1.0, shooterPower + 0.10);
        }

        // Decrease power (L1)
        if (btnL1.wasPressed(g2.left_bumper)) {
            shooterPower = Math.max(0.0, shooterPower - 0.10);
        }

        shooter.setPower(shooterPower);

        // Auto mode assignment based on power
        if (shooterPower <= 0.01) {
            shooterMode = ShooterMode.OFF;
        } else if (shooterPower < 0.65) {
            shooterMode = ShooterMode.LOW_POWER;
        } else {
            shooterMode = ShooterMode.HIGH_POWER;
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

        // Dpad Right — Next position
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 2 : 1);
            spinDex.moveToPosition(next);
        }

        // Dpad Left — Previous position
        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            int prev = spinDex.getCurrentPosition() - (g2.triangle ? 2 : 1);

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
        telemetry.addData("Shooter Power", "%.2f", shooter.getCurrentPower());
        telemetry.addData("Shooter Vel", "%.1f t/s", shooter.getAverageVelocity());

        // Intake
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Intake Power", "%.2f", intake.getCurrentPower());

        // Pusher
        telemetry.addData("Pusher State", pusher.getState());
        telemetry.addData("Pusher Servo", "%.3f", pusher.getServoPosition());

        // Spindex
        telemetry.addData("Spindex Pos", "%d/%d", spinDex.getCurrentPosition(), SPINDEX_MAX_POSITIONS);
        telemetry.addData("Spindex Servo", "%.3f", spinDex.getServoPosition());

        telemetry.addLine("Slots:");
        telemetry.addData("0", spinDex.getSlot(0));
        telemetry.addData("1", spinDex.getSlot(1));
        telemetry.addData("2", spinDex.getSlot(2));

        telemetry.addData("Filled Count", spinDex.getFilledCount());

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
    }
}
