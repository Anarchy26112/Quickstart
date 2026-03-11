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

    private static final double HOLDING_INTAKE_POWER = 1.0;
    private static final double HOLDING_TRANSFER_POWER = 0.333;

    private static final double POWER_INTAKE_POWER = 1.0;
    private static final double POWER_TRANSFER_POWER = 1.0;

    private double shooterVelocity = 0.0;
    private boolean autoShooterVelocity = true; // manual only now
    private boolean autoAlignEnabled = false;

    private static final double TARGET_X = -72;
    private static final double TARGET_Y = 144.0;
    private double distanceToTarget = 0.0;

    private static final double VEL_A = 0.0611;
    private static final double VEL_B = -7.984;
    /*
    private static final double VEL_A = 0.04541;
    private static final double VEL_B = -5.7004;
     */
    private static final double VEL_C = 1950; // 2146.31
    private static final double AUTO_ALIGN_TOLERANCE_DEGREES = 0.8;

    private static final long GATE_OPEN_DELAY_MS = 333;
    private long autoAlignEnabledAtMs = 0;
    private boolean gateOpenedAfterAutoAlignDelay = false;

    private Gamepad.RumbleEffect fastPulseEffect;
    private long nextPulseAllowedMs = 0;
    private static final int PULSE_INTERVAL_MS = 250;

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

        fastPulseEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .addStep(1.0, 1.0, 70)
                .build();
    }

    public void setAutoAlignEnabled(boolean enabled) {
        if (autoAlignEnabled == enabled) return;

        autoAlignEnabled = enabled;

        if (!enabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;

            autoAlignEnabledAtMs = 0;
            gateOpenedAfterAutoAlignDelay = false;

            // Auto align off -> POWER + gate closed
            setIntakeTransferState(IntakeTransferState.POWER);
            gate.block();
        } else {
            autoAlignEnabledAtMs = System.currentTimeMillis();
            gateOpenedAfterAutoAlignDelay = false;

            // Auto align on -> HOLD + gate stays closed for 1 second
            setIntakeTransferState(IntakeTransferState.HOLD);
            gate.block();
        }
    }

    public void update(Gamepad g2) {
        updateDistanceToTarget();
        updateGateDelay();
        handleShooter(g2);
        handleAutoAlignRumble(g2);
        handleIntakeAndTransfer(g2);
    }

    private void updateGateDelay() {
        if (!autoAlignEnabled) return;

        if (!gateOpenedAfterAutoAlignDelay &&
                System.currentTimeMillis() - autoAlignEnabledAtMs >= GATE_OPEN_DELAY_MS) {
            gate.open();
            gateOpenedAfterAutoAlignDelay = true;
        }
    }

    private void handleAutoAlignRumble(Gamepad g2) {
        if (!autoAlignEnabled) {
            g2.stopRumble();
            return;
        }

        boolean centered = limelight.isCenteredOnTarget(AUTO_ALIGN_TOLERANCE_DEGREES);

        if (centered) {
            long now = System.currentTimeMillis();
            if (now >= nextPulseAllowedMs) {
                g2.runRumbleEffect(fastPulseEffect);
                nextPulseAllowedMs = now + PULSE_INTERVAL_MS;
            }
        } else {
            g2.stopRumble();
            nextPulseAllowedMs = 0;
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
        if (autoAlignEnabled) {
            if (btnR1.wasPressed(g2.right_bumper)) {
                setIntakeTransferState(IntakeTransferState.POWER);
                gate.open();
                gateOpenedAfterAutoAlignDelay = true;
            }
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

        boolean shouldBlockGate = !autoAlignEnabled || !gateOpenedAfterAutoAlignDelay;
        if (shouldBlockGate) {
            gate.block();
        } else {
            gate.open();
        }

        switch (newState) {
            case HOLD:
                intake.intake(HOLDING_INTAKE_POWER);
                intake.transferOut(HOLDING_TRANSFER_POWER);
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

        if (autoAlignEnabled && gateOpenedAfterAutoAlignDelay) {
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
        telemetry.addData("Auto Velocity", false);
        telemetry.addData("Centered", limelight.isCenteredOnTarget(AUTO_ALIGN_TOLERANCE_DEGREES));
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        intakeTransferState = null;
    }
}