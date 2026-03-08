package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HIGH_VELOCITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.LOW_VELOCITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.SHOOTER_MAX_VELOCITY;

public class OperatorControls {

    private final Follower follower;
    private final Intake intake;
    private final Gate gate;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private String userFeedback = "";
    private long feedbackTimer = 0;
    private static final int FEEDBACK_DISPLAY_MS = 2000;

    private static final long GATE_OPEN_DELAY_MS = 1000;
    private long autoAlignStartTime = 0;
    private boolean waitingToOpenGate = false;

    private enum IntakeTransferState {
        HOLD,
        POWER
    }

    // null = OFF
    private IntakeTransferState intakeTransferState = null;

    private enum ShooterMode {
        OFF,
        LOW_VELOCITY,
        HIGH_VELOCITY
    }

    private ShooterMode shooterMode = ShooterMode.OFF;

    private static final double HOLDING_INTAKE_POWER = 0.7;
    private static final double HOLDING_TRANSFER_POWER = 0.0;

    private static final double POWER_INTAKE_POWER = 1.0;
    private static final double POWER_TRANSFER_POWER = 1.0;

    private double shooterVelocity = 0.0;
    private boolean autoShooterVelocity = true;
    private boolean autoAlignEnabled = false;

    private static final double TARGET_X = 72;
    private static final double TARGET_Y = -144.0;
    private double distanceToTarget = 0.0;

    private static final double VEL_A = 0.04541;
    private static final double VEL_B = -5.7004;
    private static final double VEL_C = 2146.31;
    private static final double AUTO_ALIGN_TOLERANCE_DEGREES = 0.9;
    private static final long AUTO_ALIGN_CENTERED_DEBOUNCE_MS = 150;

    private long centeredSinceMs = 0;

    private final ButtonHelper btnL1 = new ButtonHelper();
    private final ButtonHelper btnR1 = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    public OperatorControls(Follower follower,
                            Intake intake,
                            Shooter shooter,
                            Telemetry telemetry,
                            Limelight limelight,
                            Gate gate) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.gate = gate;

        // Default startup behavior: auto align off -> POWER + gate closed
        setIntakeTransferState(IntakeTransferState.POWER);
        gate.block();
    }

    public void setAutoAlignEnabled(boolean enabled) {
        autoAlignEnabled = enabled;
        centeredSinceMs = 0;

        if (!enabled) {
            waitingToOpenGate = false;
            autoAlignStartTime = 0;

            gate.block();
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;

            // Auto align off -> POWER mode
            setIntakeTransferState(IntakeTransferState.POWER);
        } else {
            // Auto align on -> HOLD mode immediately, gate stays closed for 1 sec
            setIntakeTransferState(IntakeTransferState.HOLD);
            gate.block();

            autoAlignStartTime = System.currentTimeMillis();
            waitingToOpenGate = true;
        }
    }

    public void update(Gamepad g2) {
        updateDistanceToTarget();
        handleDelayedGateOpen();
        handleShooter(g2);
        handleAutoAlignIntakeMode();
        handleIntakeAndTransfer(g2);
    }
    private void handleAutoAlignIntakeMode() {
        if (!autoAlignEnabled) return;

        boolean centered = limelight.isCenteredOnTarget(AUTO_ALIGN_TOLERANCE_DEGREES)
                && limelight.isTargetVisible();

        long now = System.currentTimeMillis();

        if (centered) {
            if (centeredSinceMs == 0) {
                centeredSinceMs = now;
            }

            if (now - centeredSinceMs >= AUTO_ALIGN_CENTERED_DEBOUNCE_MS && !waitingToOpenGate) {
                if (intakeTransferState != IntakeTransferState.POWER) {
                    setIntakeTransferState(IntakeTransferState.POWER);
                }
            }
        } else {
            centeredSinceMs = 0;

            if (intakeTransferState != IntakeTransferState.HOLD) {
                setIntakeTransferState(IntakeTransferState.HOLD);
            }
        }
    }

    private void handleDelayedGateOpen() {
        if (waitingToOpenGate &&
                System.currentTimeMillis() - autoAlignStartTime >= GATE_OPEN_DELAY_MS) {
            gate.open();
            waitingToOpenGate = false;
        }
    }

    private void updateDistanceToTarget() {
        Pose pose = follower.getPose();
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);
    }

    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;

        if (v < 0.0) v = 0.0;
        if (v > SHOOTER_MAX_VELOCITY) v = SHOOTER_MAX_VELOCITY;

        return v;
    }

    private void handleShooter(Gamepad g2) {
        if (!autoAlignEnabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;
            return;
        }

        if (btnOptions.wasPressed(g2.options) && g2.triangle) {
            autoShooterVelocity = !autoShooterVelocity;
            userFeedback = "Auto Velocity: " + (autoShooterVelocity ? "ON" : "OFF");
            feedbackTimer = System.currentTimeMillis();
        }

        if (autoShooterVelocity) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
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

    private void handleIntakeAndTransfer(Gamepad g2) {
        if (autoAlignEnabled) return;

        if (btnL1.wasPressed(g2.left_bumper)) {
            toggleIntakeTransferState(IntakeTransferState.HOLD);
        }

        if (btnR1.wasPressed(g2.right_bumper)) {
            toggleIntakeTransferState(IntakeTransferState.POWER);
        }
    }

    private void toggleIntakeTransferState(IntakeTransferState requestedState) {
        if (intakeTransferState == requestedState) {
            stopIntakeTransfer();
        } else {
            setIntakeTransferState(requestedState);
        }
    }

    private void setIntakeTransferState(IntakeTransferState newState) {
        intakeTransferState = newState;

        boolean shouldBlockGate = !(autoAlignEnabled && newState == IntakeTransferState.POWER);
        if (shouldBlockGate) {
            gate.block();
        } else {
            gate.open();
        }

        switch (newState) {
            case HOLD:
                intake.intake(HOLDING_INTAKE_POWER);
                intake.transferIn(HOLDING_TRANSFER_POWER);
                userFeedback = "Intake: HOLD";
                break;

            case POWER:
                intake.intake(POWER_INTAKE_POWER);
                intake.transferIn(POWER_TRANSFER_POWER);
                userFeedback = "Intake: POWER";
                break;
        }

        feedbackTimer = System.currentTimeMillis();
    }

    private void stopIntakeTransfer() {
        intakeTransferState = null;
        intake.stopAll();

        if (autoAlignEnabled) {
            gate.open();
        } else {
            gate.block();
        }

        userFeedback = "Intake: OFF";
        feedbackTimer = System.currentTimeMillis();
    }

    public void updateTelemetry() {
        limelight.displayTelemetry();

        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        telemetry.addData("Intake State", intakeTransferState == null ? "OFF" : intakeTransferState);

        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Actual Vel", "%.0f t/s", shooter.getAverageVelocity());

        telemetry.addData("Right Velocity", "%.0f", shooter.getRightVelocity());
        telemetry.addData("Left Velocity", "%.0f", shooter.getLeftVelocity());
        telemetry.addData("Distance To Target", "%.2f", distanceToTarget);
        telemetry.addData("Auto Align", autoAlignEnabled);
        telemetry.addData("Auto Velocity", autoShooterVelocity);
        telemetry.addData("Waiting Gate Open", waitingToOpenGate);
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        intakeTransferState = null;
        waitingToOpenGate = false;
    }
}