package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;

public class LimelightTuning {

    // Subsystems
    private final Intake intake;
    private final SpinDex spinDex;
    private final Shooter shooter;
    private final Pusher pusher;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;

    private final Limelight limelight;

    private final IntakeMacro intakeMacro;
    private final ShooterMacro shooterMacro;

    // Feedback field
    private String userFeedback = "";
    private long feedbackTimer = 0;

    // ============================================================
    // STATES
    // ============================================================

    private enum IntakeState { OFF, INTAKING, SPITTING, MACRO_RUNNING }
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { OFF, LOW_VELOCITY, HIGH_VELOCITY, MACRO_RUNNING }
    private ShooterMode shooterMode = ShooterMode.OFF;
    private double shooterVelocity = 0.0;

    // ============================================================
    // CONSTANTS
    // ============================================================

    private static final double VELOCITY_INCREMENT = SHOOTER_MAX_VELOCITY * 0.05;
    private static final double LOW_VELOCITY_THRESHOLD = HamiltonParams.LOW_VELOCITY_THRESHOLD;
    private static final double HIGH_VELOCITY_THRESHOLD = HamiltonParams.HIGH_VELOCITY_THRESHOLD;
    private static final int POSITIONS_PER_TURN = 6;
    private static final int FEEDBACK_DISPLAY_MS = 2000;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double Kd_INCREMENT = 0.0002;
    private static final double Ki_INCREMENT = 0.0001;
    private static final double Kp_INCREMENT = 0.001;
    private static final double INCREMENT = 0.001;
    private static final double SPINDEX_Kp_INCREMENT = 0.0002;
    private static final double SPINDEX_Kd_INCREMENT = 0.00002;


    // ============================================================
    // BUTTON HELPERS
    // ============================================================

    // Intake / Spindex Misc
    private final ButtonHelper btnCross = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnDpadUp = new ButtonHelper();
    private final ButtonHelper btnDpadDown = new ButtonHelper();

    // Manual Spindex
    private final ButtonHelper btnDpadLeft = new ButtonHelper();
    private final ButtonHelper btnDpadRight = new ButtonHelper();

    // Shooter / Pusher
    private final ButtonHelper btnSquare = new ButtonHelper();
    private final ButtonHelper btnL1 = new ButtonHelper();
    private final ButtonHelper btnR1 = new ButtonHelper();

    // Smart Align Triggers (repurposed for Kp adjustment)
    private final ButtonHelper btnL2 = new ButtonHelper();
    private final ButtonHelper btnR2 = new ButtonHelper();
    private final ButtonHelper btnL3 = new ButtonHelper();
    private final ButtonHelper btnR3 = new ButtonHelper();
    private final ButtonHelper btnL4 = new ButtonHelper();
    private final ButtonHelper btnR4 = new ButtonHelper();

    private final ButtonHelper btnShare = new ButtonHelper();

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public LimelightTuning(Intake intake,
                            SpinDex spinDex,
                            Shooter shooter,
                            Pusher pusher,
                            Telemetry telemetry,
                            ColorSensor colorSensor,
                            Limelight limelight) {

        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
        this.limelight = limelight;

        this.intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, telemetry);
        this.shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================

    public void update(Gamepad g2) {
        // Run all macros first
        intakeMacro.update();
        shooterMacro.update();

        // Check if macros just finished
        if (intakeMacro.isComplete()) {
            intakeState = IntakeState.OFF;
        }

        if (shooterMacro.isComplete() || shooterMacro.hasFailed()) {
            shooterMode = ShooterMode.OFF;
            if (shooterMacro.hasFailed()) {
                userFeedback = "SHOOTER MACRO FAILED: Empty Slots";
                feedbackTimer = System.currentTimeMillis();
            }
        }
        handleIntake(g2);
        handleSomething(g2);
        handleSpindexKp(g2);
        handleSpindexKd(g2);
        // Only allow shooting controls if the shooter macro isn't running
        if (!shooterMacro.isRunning()) {
            handleShooter(g2);
        } else {
            shooterMode = ShooterMode.MACRO_RUNNING;
        }

        // Adjust Kp_TURN with triggers
        handleKpAdjustment(g2);
        pusher.update();
    }

    private void handleKpAdjustment(Gamepad g2) {
        boolean rightTriggerPressed = g2.right_trigger > TRIGGER_THRESHOLD;
        boolean leftTriggerPressed = g2.left_trigger > TRIGGER_THRESHOLD;

        if (btnR2.wasPressed(rightTriggerPressed)) {
            HamiltonParams.Kd_TURN = HamiltonParams.Kd_TURN + Kd_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }

        if (btnL2.wasPressed(leftTriggerPressed)) {
            HamiltonParams.Kd_TURN = HamiltonParams.Kd_TURN - Kd_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }

    // ============================================================
    // INTAKE (MACRO & SPIT ONLY)
    // ============================================================

    private void handleIntake(Gamepad g2) {
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            HamiltonParams.Ki_TURN = HamiltonParams.Ki_TURN + Ki_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }

        // 2. Clear all slots (DPad Down)
        if (btnDpadDown.wasPressed(g2.dpad_down)) {
            HamiltonParams.Ki_TURN = HamiltonParams.Ki_TURN - Ki_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }

    private void handleShooter(Gamepad g2) {

        boolean r1Pressed = btnR1.wasPressed(g2.right_bumper);
        boolean l1Pressed = btnL1.wasPressed(g2.left_bumper);

        if (r1Pressed) {
            HamiltonParams.Kp_TURN = HamiltonParams.Kp_TURN + Kp_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
        else if (l1Pressed) {
            Kp_TURN = Kp_TURN - Kp_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }
    private void handleSomething(Gamepad g2) {
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            MIN_TURN_POWER = MIN_TURN_POWER + INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }

        // 2. Clear all slots (DPad Down)
        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            MIN_TURN_POWER = MIN_TURN_POWER - INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }
    private void handleSpindexKp(Gamepad g2) {
        if (btnR3.wasPressed(g2.circle)) {
            DEFAULT_KP = DEFAULT_KP + SPINDEX_Kp_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }

        // 2. Clear all slots (DPad Down)
        if (btnL3.wasPressed(g2.square)) {
            DEFAULT_KP = DEFAULT_KP - SPINDEX_Kp_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }

    private void handleSpindexKd(Gamepad g2) {
        if (btnR4.wasPressed(g2.triangle)) {
            DEFAULT_KD = DEFAULT_KD + SPINDEX_Kd_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }

        // 2. Clear all slots (DPad Down)
        if (btnL4.wasPressed(g2.cross)) {
            DEFAULT_KD = DEFAULT_KD - SPINDEX_Kd_INCREMENT;
            feedbackTimer = System.currentTimeMillis();
        }
    }


    public void updateTelemetry() {

        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }
        telemetry.addData("Kd_TURN", "%.4f", HamiltonParams.Kd_TURN);
        telemetry.addData("Kp_TURN", "%.4f", HamiltonParams.Kp_TURN);
        telemetry.addData("Ki_TURN", "%.4f", HamiltonParams.Ki_TURN);
        telemetry.addData("Spindex Kp", "%.4f", DEFAULT_KP);
        telemetry.addData("Spindex Kd", "%.5f", DEFAULT_KD);
    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        intakeMacro.stop();
        shooterMacro.stop();
    }
}