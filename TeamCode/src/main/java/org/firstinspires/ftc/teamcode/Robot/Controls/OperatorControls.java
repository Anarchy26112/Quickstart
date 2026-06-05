package org.firstinspires.ftc.teamcode.Robot.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.ButtonHelper;
import org.firstinspires.ftc.teamcode.Robot.GoalAimController;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class OperatorControls {

    private final Intake            intake;
    private final Gate              gate;
    private final Shooter           shooter;
    private final Telemetry         telemetry;
    private final GoalAimController aimController;

    private String userFeedback  = "";
    private long   feedbackTimer = 0;
    private static final int FEEDBACK_DISPLAY_MS = 2000;

    private byte intakeTransferState = 0;  // 0=INTAKING, 1=HOLDING, 2=SHOOTING
    private static final byte STATE_INTAKING = 0;
    private static final byte STATE_HOLDING  = 1;
    private static final byte STATE_SHOOTING = 2;

    private static final String[] STATE_FEEDBACK_STRINGS = {
            "State: INTAKING", "State: HOLDING", "State: SHOOTING"
    };

    private static final double INTAKING_INTAKE_POWER   = 1.0;
    private static final double INTAKING_TRANSFER_POWER = 0.4;
    private static final double HOLDING_INTAKE_POWER    = 0.0;
    private static final double HOLDING_TRANSFER_POWER  = 0.0;
    private static final double SHOOTING_INTAKE_POWER   = 1.0;
    private static final long   SHOOTING_DURATION_MS    = 600;

    private static final double MANUAL_VEL_ADJUST_RATE  = 600.0;

    private long    actualShootingStartedAtMs = 0;
    private boolean waitingToStartShooting    = false;
    private boolean autoAlignEnabled          = false;
    private boolean requestAutoAlignDisable   = false;

    // Used by TeleopBlue to force one immediate voltage refresh
    // when the feed motors actually start for shooting.
    private boolean justStartedShooting = false;

    private double distanceToTarget     = 0.0;
    private double lookupTargetVelocity = 1700.0;
    private double manualTargetVelocity = 1700.0;
    private boolean autoShooterVelocity = true;

    private long lastDistanceUpdateNs = 0;
    private static final long DISTANCE_UPDATE_INTERVAL_NS = 40_000_000L;

    private static final double VEL_A = 0.0522;
    private static final double VEL_B = -5.45;
    private static final double VEL_C = 1700.0;

    private static final double SHOOTER_RAMP_UP_RATE   = 3000.0;
    private static final double SHOOTER_RAMP_DOWN_RATE = 2500.0;

    private double commandedShooterVelocity = 0.0;
    private double lastSentShooterVelocity  = -999.0;

    private final ButtonHelper btnRightBumper = new ButtonHelper();
    private final ButtonHelper btnOptions     = new ButtonHelper();

    public OperatorControls(
            Intake intake,
            Shooter shooter,
            Telemetry telemetry,
            Gate gate,
            GoalAimController aimController
    ) {
        this.intake        = intake;
        this.shooter       = shooter;
        this.telemetry     = telemetry;
        this.gate          = gate;
        this.aimController = aimController;

        setState(STATE_INTAKING, false, 0);
        gate.block();
    }

    public void setAutoAlignEnabled(boolean enabled) {
        if (autoAlignEnabled == enabled) return;

        autoAlignEnabled          = enabled;
        waitingToStartShooting    = false;
        actualShootingStartedAtMs = 0;
        justStartedShooting       = false;

        if (enabled) {
            gate.open();
            setState(STATE_HOLDING, false, 0);
        } else {
            gate.block();
            setState(STATE_INTAKING, false, 0);
        }
    }

    public boolean shouldDisableAutoAlign() {
        return requestAutoAlignDisable;
    }

    public void clearDisableAutoAlignRequest() {
        requestAutoAlignDisable = false;
    }

    public boolean isIntaking() {
        return intakeTransferState == STATE_INTAKING;
    }

    public boolean isShooting() {
        return intakeTransferState == STATE_SHOOTING;
    }

    public boolean consumeJustStartedShooting() {
        if (!justStartedShooting) return false;

        justStartedShooting = false;
        return true;
    }

    public void update(
            Gamepad g2,
            double poseX,
            double poseY,
            long nowMs,
            long nowNs,
            double loopDtSec
    ) {
        updateDistanceToTarget(poseX, poseY, nowNs);
        updateIntakeStateMachine(g2, nowMs);

        if (intakeTransferState == STATE_SHOOTING && waitingToStartShooting) {
            intake.intakeBoth(SHOOTING_INTAKE_POWER);

            waitingToStartShooting    = false;
            actualShootingStartedAtMs = nowMs;
            justStartedShooting       = true;
        }

        updateShooterCommand(g2, nowMs, loopDtSec);
    }

    private void updateIntakeStateMachine(Gamepad g2, long nowMs) {
        if (btnRightBumper.wasPressed(g2.right_bumper) && autoAlignEnabled) {
            setState(STATE_SHOOTING, true, nowMs);

            intake.intakeBoth(SHOOTING_INTAKE_POWER);

            waitingToStartShooting    = false;
            actualShootingStartedAtMs = nowMs;
            justStartedShooting       = true;

            userFeedback  = "ACTION: FORCE FIRE";
            feedbackTimer = nowMs;

            return;
        }

        if (intakeTransferState == STATE_SHOOTING) {
            if (waitingToStartShooting || nowMs - actualShootingStartedAtMs < SHOOTING_DURATION_MS) {
                return;
            }

            requestAutoAlignDisable = true;
            setState(STATE_INTAKING, true, nowMs);

            return;
        }

        final byte desired = autoAlignEnabled ? STATE_HOLDING : STATE_INTAKING;

        if (intakeTransferState != desired) {
            setState(desired, true, nowMs);
        }
    }

    private void setState(byte newState, boolean showFeedback, long nowMs) {
        if (intakeTransferState == newState) return;

        intakeTransferState = newState;

        switch (newState) {
            case STATE_INTAKING:
                waitingToStartShooting = false;
                intake.intake(INTAKING_INTAKE_POWER);
                intake.transferIn(INTAKING_TRANSFER_POWER);
                break;

            case STATE_HOLDING:
                waitingToStartShooting = false;
                intake.intake(HOLDING_INTAKE_POWER);
                intake.transferOut(HOLDING_TRANSFER_POWER);
                break;

            case STATE_SHOOTING:
                intake.stopAll();
                waitingToStartShooting = true;
                break;
        }

        if (showFeedback) {
            userFeedback  = STATE_FEEDBACK_STRINGS[newState];
            feedbackTimer = nowMs;
        }
    }

    private void updateShooterCommand(Gamepad g2, long nowMs, double dt) {
        if (btnOptions.wasPressed(g2.options) && g2.triangle) {
            autoShooterVelocity = !autoShooterVelocity;

            userFeedback  = autoShooterVelocity ? "Auto Velocity: ON" : "Auto Velocity: OFF";
            feedbackTimer = nowMs;
        }

        if (!autoShooterVelocity) {
            manualTargetVelocity += -g2.right_stick_y * MANUAL_VEL_ADJUST_RATE * dt;

            if (manualTargetVelocity < 0.0) {
                manualTargetVelocity = 0.0;
            }
        }

        final double targetVelocity = autoShooterVelocity ? lookupTargetVelocity : manualTargetVelocity;

        if (commandedShooterVelocity != targetVelocity) {
            if (commandedShooterVelocity < targetVelocity) {
                commandedShooterVelocity += SHOOTER_RAMP_UP_RATE * dt;

                if (commandedShooterVelocity > targetVelocity) {
                    commandedShooterVelocity = targetVelocity;
                }
            } else {
                commandedShooterVelocity -= SHOOTER_RAMP_DOWN_RATE * dt;

                if (commandedShooterVelocity < targetVelocity) {
                    commandedShooterVelocity = targetVelocity;
                }
            }

            if (shouldUpdateShooterVelocity(commandedShooterVelocity, lastSentShooterVelocity)) {
                shooter.setVelocity(commandedShooterVelocity);
                lastSentShooterVelocity = commandedShooterVelocity;
            }
        }
    }

    private boolean shouldUpdateShooterVelocity(double newVel, double lastVel) {
        double diff = newVel - lastVel;
        return diff > SHOOTER_COMMAND_EPSILON || diff < -SHOOTER_COMMAND_EPSILON;
    }

    private void updateDistanceToTarget(double poseX, double poseY, long nowNs) {
        if (!autoShooterVelocity) return;
        if (nowNs - lastDistanceUpdateNs < DISTANCE_UPDATE_INTERVAL_NS) return;

        final double dx     = aimController.getShooterTargetX() - poseX;
        final double dy     = aimController.getShooterTargetY() - poseY;
        final double distSq = dx * dx + dy * dy;

        distanceToTarget     = Math.sqrt(distSq);
        lastDistanceUpdateNs = nowNs;

        double v = VEL_A * distSq + VEL_B * distanceToTarget + VEL_C;

        if (v < 0.0) {
            v = 0.0;
        }

        lookupTargetVelocity = v;
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        gate.block();

        waitingToStartShooting   = false;
        justStartedShooting      = false;
        commandedShooterVelocity = 0.0;
        lastSentShooterVelocity  = -999.0;
    }

    public void updateTelemetry(long nowMs) {
        if (telemetry == null) return;

        if (nowMs - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("Operator", userFeedback);
        }

        telemetry.addData("Transfer State",    STATE_FEEDBACK_STRINGS[intakeTransferState]);
        telemetry.addData("Auto Shooter",      autoShooterVelocity);
        telemetry.addData("Target Distance",   distanceToTarget);
        telemetry.addData("Lookup Velocity",   lookupTargetVelocity);
        telemetry.addData("Commanded Shooter", commandedShooterVelocity);

        telemetry.addData("Shooter Target X",  aimController.getShooterTargetX());
        telemetry.addData("Shooter Target Y",  aimController.getShooterTargetY());
    }
}