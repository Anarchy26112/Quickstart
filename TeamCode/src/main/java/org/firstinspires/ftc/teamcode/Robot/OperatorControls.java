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
    private final Gate gate;
    private final Shooter shooter;
    private final Telemetry telemetry;
    private final Limelight limelight;

    // Feedback field
    private String userFeedback = "";
    private long feedbackTimer = 0;

    // ============================================================
    // STATES
    // ============================================================

    private enum IntakeState { OFF, INTAKING, TRANSFER}
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { OFF, LOW_VELOCITY, HIGH_VELOCITY }
    private ShooterMode shooterMode = ShooterMode.OFF;
    private double shooterVelocity = 0.0;

    // ============================================================
    // INTAKE/TRANSFER POWER TUNING (runtime adjustable)
    // NOTE: Intake subsystem must support: intake.run(double intakePwr, double transferPwr)
    // ============================================================

    private double intakePower = INTAKE_POWER;       // starts at defaults (can go negative)
    private double transferPower = TRANSFER_POWER;   // starts at defaults (can go negative)

    private static final double POWER_STEP = 0.05;   // 5% per press
    private static final double TRIGGER_PRESS_THRESHOLD = 0.6;

    private double clampPower(double p) {
        if (p > 1.0) return 1.0;
        if (p < -1.0) return -1.0;
        return p;
    }

    // ============================================================
    // CONSTANTS
    // ============================================================

    private static final double VELOCITY_INCREMENT = SHOOTER_MAX_VELOCITY * 0.05;
    private static final double LOW_VELOCITY_THRESHOLD = HamiltonParams.LOW_VELOCITY_THRESHOLD;
    private static final double HIGH_VELOCITY_THRESHOLD = HamiltonParams.HIGH_VELOCITY_THRESHOLD;

    // Distance-to-point target (field units must match Pose units)
    private static final double TARGET_X = 72;
    private static final double TARGET_Y = -144.0;

    private double distanceToTarget = 0.0;

    // ============================================================
    // DISTANCE -> VELOCITY BEST-FIT (Quadratic)
    // v = a*d^2 + b*d + c
    // ============================================================

    private static final double VEL_A = 0.04541;
    private static final double VEL_B = -5.7004;
    private static final double VEL_C = 2146.31;

    // Toggle distance-based auto velocity
    private boolean autoShooterVelocity = true;
    private static final int FEEDBACK_DISPLAY_MS = 2000;
    private boolean autoAlignEnabled = false;

    public void setAutoAlignEnabled(boolean enabled) {
        this.autoAlignEnabled = enabled;

        if (enabled) {
            gate.open();   // <-- NEW: open immediately when auto-align turns on
        } else {
            gate.block();  // <-- optional but recommended: return to safe state
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;
        }
    }

    // ============================================================
    // BUTTON HELPERS
    // ============================================================

    // Gate toggle
    private final ButtonHelper btnCross = new ButtonHelper();

    // Intake toggle (you currently use dpad_up)
    private final ButtonHelper btnDpadUp = new ButtonHelper();
    private final ButtonHelper btnDpadDown = new ButtonHelper();

    private final ButtonHelper btnTriangle = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();

    // Intake Power tuning (DPAD LEFT/RIGHT)
    private final ButtonHelper btnDpadLeft = new ButtonHelper();
    private final ButtonHelper btnDpadRight = new ButtonHelper();

    // Shooter / velocity controls
    private final ButtonHelper btnSquare = new ButtonHelper();
    private final ButtonHelper btnL1 = new ButtonHelper();
    private final ButtonHelper btnR1 = new ButtonHelper();

    // Transfer Power tuning (TRIGGERS)
    private final ButtonHelper btnL2 = new ButtonHelper();
    private final ButtonHelper btnR2 = new ButtonHelper();

    private final ButtonHelper btnOptions = new ButtonHelper();

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

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
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================

    public void update(Gamepad g2) {
        Pose pose = follower.getPose();
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);

        // If auto-align is NOT enabled, shooter must be OFF no matter what
        if (!autoAlignEnabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0); // or shooter.stop()
            shooterMode = ShooterMode.OFF;
        }

        // Auto distance-based shooter velocity (only when enabled)
        if (autoAlignEnabled && autoShooterVelocity) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
        }

        // NEW: adjust intake/transfer powers with dpads + triggers
        handleIntakeTransferPowerTuning(g2);

        handleIntake(g2);

        if (autoAlignEnabled && intakeState != IntakeState.INTAKING) {
            gate.open();
        }

        handleShooter(g2);
    }

    // ============================================================
    // DISTANCE -> VELOCITY FUNCTION
    // ============================================================

    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;

        // Clamp to valid shooter range
        if (v < 0.0) v = 0.0;
        if (v > SHOOTER_MAX_VELOCITY) v = SHOOTER_MAX_VELOCITY;

        return v;
    }

    // ============================================================
    // INTAKE/TRANSFER POWER TUNING
    // DPAD LEFT/RIGHT -> intakePower +/- 5%
    // L2/R2 -> transferPower -/+ 5% (can go negative)
    // ============================================================

    private void handleIntakeTransferPowerTuning(Gamepad g2) {
        // Intake power: DPAD left/right
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            intakePower = clampPower(intakePower + POWER_STEP);
            userFeedback = String.format("Intake Power: %.0f%%", intakePower * 100.0);
            feedbackTimer = System.currentTimeMillis();
        } else if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            intakePower = clampPower(intakePower - POWER_STEP);
            userFeedback = String.format("Intake Power: %.0f%%", intakePower * 100.0);
            feedbackTimer = System.currentTimeMillis();
        }

        // Transfer power: DPAD up/down
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            transferPower = clampPower(transferPower + POWER_STEP);
            userFeedback = String.format("Transfer Power: %.0f%%", transferPower * 100.0);
            feedbackTimer = System.currentTimeMillis();
        } else if (btnDpadDown.wasPressed(g2.dpad_down)) {
            transferPower = clampPower(transferPower - POWER_STEP);
            userFeedback = String.format("Transfer Power: %.0f%%", transferPower * 100.0);
            feedbackTimer = System.currentTimeMillis();
        }

        // If currently running intake, apply changes immediately
        if (intakeState == IntakeState.INTAKING) {
            intake.intake(intakePower);
            intake.transferIn(transferPower);
            gate.block();
        }
    }

    // ============================================================
    // INTAKE (MANUAL TOGGLE)
    // ============================================================

    private void handleIntake(Gamepad g2) {
        if (btnTriangle.wasPressed(g2.triangle)) {
            if (intakeState == IntakeState.INTAKING) {
                // Turn OFF
                intake.stopAll();
                intakeState = IntakeState.OFF;
            } else {
                // Turn ON (custom powers)
                gate.block();  // close gate
                intake.intake(intakePower);
                intake.transferIn(transferPower);
                intakeState = IntakeState.INTAKING;
            }
            return;
        }

        // CIRCLE: run intakeBoth() and mark as TRANSFER state
        if (btnCircle.wasPressed(g2.circle)) {
            intake.intakeBoth();   // your combined behavior
            intakeState = IntakeState.TRANSFER;
            return;
        }
    }

    // ============================================================
    // SHOOTER HANDLING (MANUAL)
    // ============================================================

    private void handleShooter(Gamepad g2) {
        // Shooter disabled unless auto-align is enabled
        if (!autoAlignEnabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0); // or shooter.stop()
            shooterMode = ShooterMode.OFF;
            return;
        }

        boolean r1Pressed = btnR1.wasPressed(g2.right_bumper);
        boolean l1Pressed = btnL1.wasPressed(g2.left_bumper);
        boolean triangleHeld = g2.triangle;

        // Toggle auto velocity on/off (OPTIONS + TRIANGLE)
        if (btnOptions.wasPressed(g2.options) && triangleHeld) {
            autoShooterVelocity = !autoShooterVelocity;
            userFeedback = "Auto Velocity: " + (autoShooterVelocity ? "ON" : "OFF");
            feedbackTimer = System.currentTimeMillis();
        }

        // Manual controls only when auto is OFF
        if (!autoShooterVelocity) {
            // Triangle + bumper = presets
            if (triangleHeld && r1Pressed) {
                shooterVelocity = SHOOTER_MAX_VELOCITY * 0.71;
                feedbackTimer = System.currentTimeMillis();
            } else if (triangleHeld && l1Pressed) {
                shooterVelocity = 0;
                feedbackTimer = System.currentTimeMillis();
            }
            // No triangle: bumpers nudge by +/- 5
            else if (r1Pressed) {
                shooterVelocity += 5.0;
                userFeedback = "Shooter: +5";
                feedbackTimer = System.currentTimeMillis();
            } else if (l1Pressed) {
                shooterVelocity -= 5.0;
                userFeedback = "Shooter: -5";
                feedbackTimer = System.currentTimeMillis();
            }

            // Clamp
            if (shooterVelocity < 0.0) shooterVelocity = 0.0;
            if (shooterVelocity > SHOOTER_MAX_VELOCITY) shooterVelocity = SHOOTER_MAX_VELOCITY;
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
        telemetry.addData("Left Velocity", shooter.getLeftVelocity());

        telemetry.addData("Intake Power", "%.2f", intakePower);
        telemetry.addData("Transfer Power", "%.2f", transferPower);
    }

    public void stopAll() {
        intake.stopAll();
        shooter.stop();
    }
}