package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.SHOOTER_MAX_VELOCITY;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class OperatorControls {

    private final Intake intake;
    private final Gate gate;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final GoalAimController aimController;

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
    private static final double INTAKING_TRANSFER_POWER = 0.4;

    private static final double HOLDING_INTAKE_POWER = 0.2;
    private static final double HOLDING_TRANSFER_POWER = 0.0;

    private static final double SHOOTING_INTAKE_POWER = 1.0;
    private static final double SHOOTING_TRANSFER_POWER = 1.0;

    private static final long SHOOTING_START_DELAY_MS = 0;
    private static final long SHOOTING_DURATION_MS = 600;
    private static final double ROBOT_STOPPED_SPEED_THRESHOLD = 4.5;

    private long shootingRequestedAtMs = 0;
    private long shootingStateStartedAtMs = 0;
    private long actualShootingStartedAtMs = 0;
    private boolean waitingToStartShooting = false;

    private boolean autoAlignEnabled = false;
    private boolean autoFireLatched = false;
    private boolean requestAutoAlignDisable = false;
    private boolean requestAutoAlignEnable = false;

    private double distanceToTarget = 0.0;
    private double shooterVelocity = 0.0;
    private boolean autoShooterVelocity = true;
    private double lastRobotSpeed = 0.0;

    private static final double VEL_A = 0.049;
    private static final double VEL_B = -5.684;
    private static final double VEL_C = 1700.0;
    private static final double SHOOTER_IDLE_VELOCITY = 700.0;
    private static final double SHOOTER_RAMP_UP_RATE = 1650.0;
    private static final double SHOOTER_RAMP_DOWN_RATE = 2500.0;
    private static final double SHOOTER_MIN_COMMAND_VELOCITY = 0.0;

    private double commandedShooterVelocity = 0.0;
    private long lastShooterUpdateNs = 0;

    private final ButtonHelper btnRightBumper = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    public OperatorControls(
            Intake intake,
            Shooter shooter,
            Telemetry telemetry,
            Gate gate,
            GoalAimController aimController,
            double targetX,
            double targetY
    ) {
        this.intake = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.gate = gate;
        this.aimController = aimController;
        this.targetX = targetX;
        this.targetY = targetY;

        setState(IntakeTransferState.INTAKING, false, 0);
    }

    public void setAutoAlignEnabled(boolean enabled) {
        if (autoAlignEnabled == enabled) return;

        autoAlignEnabled = enabled;

        waitingToStartShooting = false;
        shootingRequestedAtMs = 0;
        shootingStateStartedAtMs = 0;
        actualShootingStartedAtMs = 0;
        autoFireLatched = false;

        if (autoAlignEnabled) {
            setState(IntakeTransferState.HOLDING, false, 0);
        } else {
            setState(IntakeTransferState.INTAKING, false, 0);
        }
    }

    public boolean shouldEnableAutoAlign() {
        return requestAutoAlignEnable;
    }

    public void clearEnableAutoAlignRequest() {
        requestAutoAlignEnable = false;
    }

    public boolean shouldDisableAutoAlign() {
        return requestAutoAlignDisable;
    }

    public void clearDisableAutoAlignRequest() {
        requestAutoAlignDisable = false;
    }

    public void update(Gamepad g2, Pose pose, Vector vel, long nowMs, long nowNs) {
        updateDistanceToTarget(pose);
        lastRobotSpeed = getRobotSpeed(vel);

        if (pose != null) {
            shooter.setRobotY(pose.getY());
        }

        updateIntakeStateMachine(g2, nowMs);
        updateDelayedShootingStart(nowMs);
        updateShooterCommand(g2, nowMs, nowNs);
        syncGateToMode();
    }

    private void updateIntakeStateMachine(Gamepad g2, long nowMs) {
        boolean rightBumperPressed = btnRightBumper.wasPressed(g2.right_bumper);

        // Right bumper forces shooter mode whenver Auto Align is enabled
        if (rightBumperPressed && autoAlignEnabled) {
            triggerShoot(nowMs, true);
            autoFireLatched = true;
            return;
        }

        boolean aimShootReady =
                aimController != null && aimController.isShootReady();

        boolean shootReadyAndStopped =
                autoAlignEnabled
                        && aimShootReady
                        && lastRobotSpeed < ROBOT_STOPPED_SPEED_THRESHOLD;

        if (!shootReadyAndStopped) {
            autoFireLatched = false;
        }

        if (intakeTransferState == IntakeTransferState.SHOOTING) {
            if (waitingToStartShooting) return;

            if (nowMs - actualShootingStartedAtMs < SHOOTING_DURATION_MS) {
                return;
            }

            requestAutoAlignDisable = true;
            setState(IntakeTransferState.INTAKING, true, nowMs);
            return;
        }

        if (autoAlignEnabled
                && intakeTransferState == IntakeTransferState.HOLDING
                && shootReadyAndStopped
                && !autoFireLatched) {
            triggerShoot(nowMs, false);
            autoFireLatched = true;
            return;
        }

        if (autoAlignEnabled) {
            setState(IntakeTransferState.HOLDING, true, nowMs);
        } else {
            setState(IntakeTransferState.INTAKING, true, nowMs);
        }
    }

    private void triggerShoot(long nowMs, boolean forceFeedNow) {
        setState(IntakeTransferState.SHOOTING, true, nowMs);
        shootingStateStartedAtMs = nowMs;

        if (forceFeedNow) {
            intake.intake(SHOOTING_INTAKE_POWER);
            intake.transferIn(SHOOTING_TRANSFER_POWER);
            waitingToStartShooting = false;
            actualShootingStartedAtMs = nowMs;
            userFeedback = "ACTION: FORCE FIRE";
            feedbackTimer = nowMs;
        }
    }

    private void setState(IntakeTransferState newState, boolean showFeedback, long nowMs) {
        if (intakeTransferState == newState) return;

        intakeTransferState = newState;

        switch (newState) {
            case INTAKING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;
                shootingStateStartedAtMs = 0;
                actualShootingStartedAtMs = 0;
                intake.intake(INTAKING_INTAKE_POWER);
                intake.transferIn(INTAKING_TRANSFER_POWER);
                break;

            case HOLDING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;
                shootingStateStartedAtMs = 0;
                actualShootingStartedAtMs = 0;
                intake.intake(HOLDING_INTAKE_POWER);
                intake.transferOut(HOLDING_TRANSFER_POWER);
                break;

            case SHOOTING:
                intake.stopAll();
                waitingToStartShooting = true;
                shootingRequestedAtMs = nowMs;
                break;
        }

        if (showFeedback) {
            userFeedback = "State: " + newState.name();
            feedbackTimer = nowMs;
        }
    }

    private void updateDelayedShootingStart(long nowMs) {
        if (intakeTransferState != IntakeTransferState.SHOOTING) return;
        if (!waitingToStartShooting) return;

        if (nowMs - shootingRequestedAtMs >= SHOOTING_START_DELAY_MS) {
            intake.intake(SHOOTING_INTAKE_POWER);
            intake.transferIn(SHOOTING_TRANSFER_POWER);
            waitingToStartShooting = false;
            actualShootingStartedAtMs = nowMs;
        }
    }

    private double getShooterVelocityForAtSpeedCheck() {
        double right = shooter.getRightVelocity();
        double left = shooter.getLeftVelocity();
        return 0.5 * (right + left);
    }

    private void updateShooterCommand(Gamepad g2, long nowMs, long nowNs) {
        if (btnOptions.wasPressed(g2.options) && g2.triangle) {
            autoShooterVelocity = !autoShooterVelocity;
            userFeedback = "Auto Velocity: " + (autoShooterVelocity ? "ON" : "OFF");
            feedbackTimer = nowMs;
        }

        if (lastShooterUpdateNs == 0) {
            lastShooterUpdateNs = nowNs;
        }

        double dt = (nowNs - lastShooterUpdateNs) / 1_000_000_000.0;
        lastShooterUpdateNs = nowNs;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.1) dt = 0.1;

        double targetVelocity = getDesiredShooterVelocity();

        targetVelocity = clamp(
                targetVelocity,
                SHOOTER_MIN_COMMAND_VELOCITY,
                SHOOTER_MAX_VELOCITY
        );

        commandedShooterVelocity = rampToTarget(
                commandedShooterVelocity,
                targetVelocity,
                dt,
                SHOOTER_RAMP_UP_RATE,
                SHOOTER_RAMP_DOWN_RATE
        );

        shooter.setVelocity(commandedShooterVelocity);
    }

    private double getDesiredShooterVelocity() {
        if (intakeTransferState == IntakeTransferState.INTAKING) {
            return SHOOTER_IDLE_VELOCITY;
        }

        if (autoShooterVelocity) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
        }

        return shooterVelocity;
    }

    private void syncGateToMode() {
        if (autoAlignEnabled) {
            gate.open();
        } else {
            gate.block();
        }
    }

    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;
        return clamp(v, 0.0, SHOOTER_MAX_VELOCITY);
    }

    private void updateDistanceToTarget(Pose pose) {
        if (pose == null) {
            distanceToTarget = 0.0;
            return;
        }

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);
    }

    private double getRobotSpeed(Vector vel) {
        return vel != null ? vel.getMagnitude() : 0.0;
    }

    public boolean isIntaking() {
        return intakeTransferState == IntakeTransferState.INTAKING;
    }

    private double rampToTarget(
            double current,
            double target,
            double dt,
            double rampUpRate,
            double rampDownRate
    ) {
        if (target > current) {
            double maxStep = rampUpRate * dt;
            return Math.min(current + maxStep, target);
        } else {
            double maxStep = rampDownRate * dt;
            return Math.max(current - maxStep, target);
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void setManualShooterVelocity(double velocity) {
        shooterVelocity = clamp(velocity, 0.0, SHOOTER_MAX_VELOCITY);
    }

    public boolean isAutoShooterVelocity() {
        return autoShooterVelocity;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public double getCommandedShooterVelocity() {
        return commandedShooterVelocity;
    }

    public void updateTelemetry(long nowMs) {
        telemetry.addData("Op Mode", autoAlignEnabled ? "AUTO_ALIGN" : "MANUAL");
        telemetry.addData("Intake State", intakeTransferState.name());
        telemetry.addData("Target Dist", "%.2f", distanceToTarget);
        telemetry.addData("Robot Speed", "%.2f", lastRobotSpeed);
        telemetry.addData("Shooter Target", "%.1f", shooterVelocity);
        telemetry.addData("Shooter Cmd", "%.1f", commandedShooterVelocity);
        telemetry.addData("Shooter Actual", "%.1f", getShooterVelocityForAtSpeedCheck());
        telemetry.addData("Auto Velocity", autoShooterVelocity);
        telemetry.addData("Waiting Shoot Start", waitingToStartShooting);

        if (aimController != null) {
            telemetry.addData("Shoot Ready", aimController.isShootReady());
            telemetry.addData("Shoot Ready Latched", aimController.isShootReadyLatched());
            telemetry.addData("Shoot Block", aimController.getShootBlockReason());
        }

        if (!userFeedback.isEmpty() && nowMs - feedbackTimer <= FEEDBACK_DISPLAY_MS) {
            telemetry.addData("Feedback", userFeedback);
        }
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        gate.block();
        waitingToStartShooting = false;
        autoFireLatched = false;
    }
}