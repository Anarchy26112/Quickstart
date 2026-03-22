package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.SHOOTER_MAX_VELOCITY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class OperatorControls {

    private final Follower follower;
    private final Intake intake;
    private final Gate gate;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private final double targetX;
    private final double targetY;

    private String userFeedback = "";
    private long feedbackTimer = 0;
    private static final int FEEDBACK_DISPLAY_MS = 2000;

    private enum IntakeTransferState {
        INTAKING,
        HOLDING,
        SHOOTING
    }

    private IntakeTransferState intakeTransferState = IntakeTransferState.INTAKING;

    private static final double INTAKING_INTAKE_POWER = 1.0;
    private static final double INTAKING_TRANSFER_POWER = 0.5;

    private static final double HOLDING_INTAKE_POWER = 0.5;

    private static final double SHOOTING_INTAKE_POWER = 1.0;
    private static final double SHOOTING_TRANSFER_POWER = 1.0;

    private static final long SHOOTING_START_DELAY_MS = 0;
    private long shootingRequestedAtMs = 0;
    private boolean waitingToStartShooting = false;

    private boolean autoAlignEnabled = false;
    private double distanceToTarget = 0.0;

    private static final long SHOOTING_DURATION_MS = 900;
    private long shootingStateStartedAtMs = 0;

    private static final double ROBOT_STOPPED_SPEED_THRESHOLD = 1.0;
    private boolean autoFireLatched = false;
    private boolean requestAutoAlignDisable = false;

    private double shooterVelocity = 0.0;
    private boolean autoShooterVelocity = true;

    private static final double VEL_A = 0.049;
    private static final double VEL_B = -5.684;
    private static final double VEL_C = 1700.0;

    private final ButtonHelper btnRightBumper = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    public OperatorControls(Follower follower,
                            Intake intake,
                            Shooter shooter,
                            Telemetry telemetry,
                            Limelight limelight,
                            Gate gate,
                            double targetX,
                            double targetY) {
        this.follower = follower;
        this.intake = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.gate = gate;
        this.targetX = targetX;
        this.targetY = targetY;

        applyState(IntakeTransferState.INTAKING, false);
    }

    public void setAutoAlignEnabled(boolean enabled) {
        if (autoAlignEnabled == enabled) return;

        autoAlignEnabled = enabled;
        waitingToStartShooting = false;
        shootingRequestedAtMs = 0;
        autoFireLatched = false;

        if (!enabled) {
            applyState(IntakeTransferState.INTAKING, false);
        } else {
            applyState(IntakeTransferState.HOLDING, false);
        }
    }

    public boolean shouldDisableAutoAlign() {
        return requestAutoAlignDisable;
    }

    public void clearDisableAutoAlignRequest() {
        requestAutoAlignDisable = false;
    }

    public void update(Gamepad g2) {
        updateDistanceToTarget();
        updateIntakeStateMachine(g2);
        updateDelayedShootingStart();
        handleShooter(g2);
    }

    private void updateIntakeStateMachine(Gamepad g2) {
        boolean rightBumperPressed = btnRightBumper.wasPressed(g2.right_bumper);

        boolean shootReadyAndStopped =
                autoAlignEnabled
                        && limelight.isShootReady()
                        && getRobotSpeed() < ROBOT_STOPPED_SPEED_THRESHOLD;

        if (!shootReadyAndStopped) {
            autoFireLatched = false;
        }

        if (autoAlignEnabled && rightBumperPressed) {
            triggerShoot();
            autoFireLatched = true;
            return;
        }

        if (intakeTransferState == IntakeTransferState.SHOOTING) {
            if (System.currentTimeMillis() - shootingStateStartedAtMs < SHOOTING_DURATION_MS) {
                return;
            } else {
                requestAutoAlignDisable = true;
                applyState(IntakeTransferState.INTAKING, true);
                return;
            }
        }

        if (autoAlignEnabled
                && intakeTransferState == IntakeTransferState.HOLDING
                && shootReadyAndStopped
                && !autoFireLatched) {
            triggerShoot();
            autoFireLatched = true;
            return;
        }

        if (!autoAlignEnabled) {
            if (intakeTransferState != IntakeTransferState.INTAKING) {
                applyState(IntakeTransferState.INTAKING, true);
            }
        } else {
            if (intakeTransferState != IntakeTransferState.HOLDING) {
                applyState(IntakeTransferState.HOLDING, true);
            }
        }
    }

    private void triggerShoot() {
        applyState(IntakeTransferState.SHOOTING, true);
        shootingStateStartedAtMs = System.currentTimeMillis();
    }

    private void applyState(IntakeTransferState newState, boolean showFeedback) {
        intakeTransferState = newState;

        switch (newState) {
            case INTAKING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;
                intake.intake(INTAKING_INTAKE_POWER);
                intake.transferIn(INTAKING_TRANSFER_POWER);
                break;

            case HOLDING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;
                intake.intake(HOLDING_INTAKE_POWER);
                intake.stopTransfer();
                break;

            case SHOOTING:
                intake.stopAll();
                waitingToStartShooting = true;
                shootingRequestedAtMs = System.currentTimeMillis();
                break;
        }

        if (autoAlignEnabled) {
            gate.open();
        } else {
            gate.block();
        }

        if (showFeedback) {
            userFeedback = "State: " + newState.name();
            feedbackTimer = System.currentTimeMillis();
        }
    }

    private void updateDelayedShootingStart() {
        if (intakeTransferState != IntakeTransferState.SHOOTING) return;
        if (!waitingToStartShooting) return;

        if (System.currentTimeMillis() - shootingRequestedAtMs >= SHOOTING_START_DELAY_MS) {
            intake.intake(SHOOTING_INTAKE_POWER);
            intake.transferIn(SHOOTING_TRANSFER_POWER);
            waitingToStartShooting = false;
        }
    }

    private void handleShooter(Gamepad g2) {
        if (btnOptions.wasPressed(g2.options) && g2.triangle) {
            autoShooterVelocity = !autoShooterVelocity;
            userFeedback = "Auto Velocity: " + (autoShooterVelocity ? "ON" : "OFF");
            feedbackTimer = System.currentTimeMillis();
        }

        if (intakeTransferState == IntakeTransferState.INTAKING) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            return;
        }

        if (autoShooterVelocity) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
        }

        shooter.setVelocity(shooterVelocity);
    }

    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;

        if (v < 0.0) v = 0.0;
        if (v > SHOOTER_MAX_VELOCITY) v = SHOOTER_MAX_VELOCITY;

        return v;
    }

    private void updateDistanceToTarget() {
        Pose pose = follower.getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);
    }

    private double getRobotSpeed() {
        Vector vel = follower.getVelocity();
        return vel.getMagnitude();
    }

    public void setManualShooterVelocity(double velocity) {
        shooterVelocity = Math.max(0.0, Math.min(velocity, SHOOTER_MAX_VELOCITY));
    }

    public boolean isAutoShooterVelocity() {
        return autoShooterVelocity;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public void updateTelemetry() {
        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        telemetry.addData("Intake State", intakeTransferState);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Distance To Target", "%.2f", distanceToTarget);
        telemetry.addData("Target Point", "(%.1f, %.1f)", targetX, targetY);
        telemetry.addData("Auto Align", autoAlignEnabled);
        telemetry.addData("Auto Velocity", autoShooterVelocity);
        telemetry.addData("Limelight Settled", limelight.isSettled());
        telemetry.addData("Limelight Shoot Ready", limelight.isShootReady());
        telemetry.addData("Robot Speed", "%.2f in/s", getRobotSpeed());
        telemetry.addData("Waiting Shoot Delay", waitingToStartShooting);
        telemetry.addData("Auto Fire Latched", autoFireLatched);
        telemetry.addData("Disable Auto Align Req", requestAutoAlignDisable);
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        gate.block();
        waitingToStartShooting = false;
        shootingRequestedAtMs = 0;
        autoFireLatched = false;
        requestAutoAlignDisable = false;
    }
}