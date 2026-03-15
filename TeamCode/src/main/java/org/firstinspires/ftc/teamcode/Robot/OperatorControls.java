package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.SHOOTER_MAX_VELOCITY;

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

    private enum ShooterMode {
        OFF,
        LOW_VELOCITY,
        HIGH_VELOCITY
    }

    private ShooterMode shooterMode = ShooterMode.OFF;

    // Intake / transfer powers
    private static final double INTAKING_INTAKE_POWER = 1.0;
    private static final double INTAKING_TRANSFER_POWER = 1.0;

    private static final double HOLDING_INTAKE_POWER = 1.0;
    private static final double HOLDING_TRANSFER_POWER = 0.0;

    private static final double SHOOTING_INTAKE_POWER = 1.0;
    private static final double SHOOTING_TRANSFER_POWER = 1.0;

    // Delay between gate opening and shooting motors starting
    private static final long SHOOTING_START_DELAY_MS = 366;
    private long shootingRequestedAtMs = 0;
    private boolean waitingToStartShooting = false;

    private double shooterVelocity = 0.0;
    private boolean autoShooterVelocity = true;
    private boolean autoAlignEnabled = false;

    private double distanceToTarget = 0.0;

    private static final double VEL_A = 0.0498;
    private static final double VEL_B = -5.684;
    private static final double VEL_C = 1700;
    private static final double AUTO_ALIGN_TOLERANCE_DEGREES = 0.9;

    private static final long SHOOTING_DURATION_MS = 2000;
    private long shootingStateStartedAtMs = 0;

    private final ButtonHelper btnOptions = new ButtonHelper();
    private final ButtonHelper btnRightBumper = new ButtonHelper();

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

        if (!enabled) {
            applyState(IntakeTransferState.INTAKING, false);
        } else {
            applyState(IntakeTransferState.HOLDING, false);
        }
    }

    public void update(Gamepad g2) {
        updateDistanceToTarget();
        updateIntakeStateMachine(g2);
        updateDelayedShootingStart();
        handleShooter(g2);
    }

    private void updateIntakeStateMachine(Gamepad g2) {
        boolean rightBumperPressed = btnRightBumper.wasPressed(g2.right_bumper);

        if (autoAlignEnabled && rightBumperPressed) {
            applyState(IntakeTransferState.SHOOTING, true);
            shootingStateStartedAtMs = System.currentTimeMillis();
            return;
        }

        if (intakeTransferState == IntakeTransferState.SHOOTING) {
            if (System.currentTimeMillis() - shootingStateStartedAtMs < SHOOTING_DURATION_MS) {
                return;
            } else {
                applyState(IntakeTransferState.HOLDING, true);
                return;
            }
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

    private void applyState(IntakeTransferState newState, boolean showFeedback) {
        intakeTransferState = newState;

        switch (newState) {
            case INTAKING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;

                gate.block();
                intake.intake(INTAKING_INTAKE_POWER);
                intake.transferIn(INTAKING_TRANSFER_POWER);

                if (showFeedback) {
                    userFeedback = "State: INTAKING";
                    feedbackTimer = System.currentTimeMillis();
                }
                break;

            case HOLDING:
                waitingToStartShooting = false;
                shootingRequestedAtMs = 0;

                gate.block();
                intake.intake(HOLDING_INTAKE_POWER);
                intake.stopTransfer();

                if (showFeedback) {
                    userFeedback = "State: HOLDING";
                    feedbackTimer = System.currentTimeMillis();
                }
                break;

            case SHOOTING:
                gate.open();
                intake.stopAll();

                waitingToStartShooting = true;
                shootingRequestedAtMs = System.currentTimeMillis();

                if (showFeedback) {
                    userFeedback = "State: SHOOTING";
                    feedbackTimer = System.currentTimeMillis();
                }
                break;
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

    private void updateDistanceToTarget() {
        Pose pose = follower.getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);
    }

    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;

        if (v < 0.0) v = 0.0;
        if (v > SHOOTER_MAX_VELOCITY) v = SHOOTER_MAX_VELOCITY;

        return v;
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
            shooterMode = ShooterMode.OFF;
            return;
        }

        if (autoShooterVelocity) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
        }

        shooter.setVelocity(shooterVelocity);
    }

    public void updateTelemetry() {
        limelight.displayTelemetry();

        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        telemetry.addData("Intake State", intakeTransferState);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Distance To Target", "%.2f", distanceToTarget);
        telemetry.addData("Target Point", "(%.1f, %.1f)", targetX, targetY);
        telemetry.addData("Auto Align", autoAlignEnabled);
        telemetry.addData("Auto Velocity", autoShooterVelocity);
        telemetry.addData("Centered", limelight.isCenteredOnTarget(AUTO_ALIGN_TOLERANCE_DEGREES));
        telemetry.addData("Waiting Shoot Delay", waitingToStartShooting);
        telemetry.addData("Test", true);
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
        gate.block();
        waitingToStartShooting = false;
        shootingRequestedAtMs = 0;
    }
}