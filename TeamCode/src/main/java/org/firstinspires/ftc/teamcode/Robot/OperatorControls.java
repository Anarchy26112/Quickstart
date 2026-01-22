package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class OperatorControls {

    private final Follower follower;
    // Subsystems
    private final Intake intake;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;

    private final Limelight limelight;

    // Feedback field
    private String userFeedback = "";
    private long feedbackTimer = 0;

    // ============================================================
    // STATES
    // ============================================================

    private enum IntakeState { OFF, INTAKING, MACRO_RUNNING }
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
    private static final int FEEDBACK_DISPLAY_MS = 2000;

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

    private final ButtonHelper btnOptions = new ButtonHelper();

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public OperatorControls(Intake intake,
                            Shooter shooter,
                            Telemetry telemetry,
                            ColorSensor colorSensor,
                            Limelight limelight, HardwareMap hardwareMap) {

        this.intake = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
        this.limelight = limelight;

        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================

    public void update(Gamepad g2) {
        // CRITICAL: Update SpinDex periodic control first (runs PD controller)
        follower.update();

        // Always handle intake controls (stop button needs to work anytime)
        handleIntake(g2);

        handleShooter(g2);
    }


    // ============================================================
    // INTAKE (MANUAL)
    // ============================================================

    private void handleIntake(Gamepad g2) {
        // 3. Toggle Manual Intake (Circle / B)
        if (btnDpadUp.wasPressed(g2.dpad_up )) {
            switch (intakeState) {
                case INTAKING:
                    intake.stopAll();
                    intakeState = IntakeState.OFF;
                    break;
                default:
                    intake.intakeBoth();
                    intakeState = IntakeState.INTAKING;
                    break;
            }
        }
    }

    // ============================================================
    // SHOOTER HANDLING (MANUAL)
    // ============================================================

    private void handleShooter(Gamepad g2) {

        boolean r1Pressed = btnR1.wasPressed(g2.right_bumper);
        boolean l1Pressed = btnL1.wasPressed(g2.left_bumper);
        boolean triangleHeld = g2.triangle;

        if (r1Pressed & triangleHeld) {
            shooterVelocity = 2255.0;
        }
        else if (l1Pressed && triangleHeld) {
            shooterVelocity = 0.0;
        }
        else if (l1Pressed) {
            shooterVelocity -= 5;
        }
        else if (r1Pressed) {
            shooterVelocity += 5;
        }

        shooter.setVelocity(shooterVelocity);

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
    // TELEMETRY
    // ============================================================

    public void updateTelemetry() {
        limelight.displayTelemetry();

        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Actual Vel", "%.0f t/s", shooter.getAverageVelocity());

        telemetry.addData("Right Velocity", shooter.getRightVelocity());
        telemetry.addData("left Velocity", shooter.getLeftVelocity());

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        telemetry.addData("P: ", follower.getPose().getX());
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
    }
}