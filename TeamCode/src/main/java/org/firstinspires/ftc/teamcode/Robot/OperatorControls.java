package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex.POSITION_TOLERANCE_TICKS;

public class OperatorControls {

    private final Follower follower;

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
    // AUTO-ALIGN -> SHOOTER ARMING
    // ============================================================
    private boolean autoAlignEnabled = false;

    public void setAutoAlignEnabled(boolean enabled) {
        this.autoAlignEnabled = enabled;

        if (!enabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;

            if (shooterMacro != null && shooterMacro.isRunning()) {
                shooterMacro.stop();
            }
        }
    }

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
    private static final double LOW_VELOCITY_THRESHOLD = HamiltonParams.LOW_VELOCITY_THRESHOLD;
    private static final double HIGH_VELOCITY_THRESHOLD = HamiltonParams.HIGH_VELOCITY_THRESHOLD;

    public static final int POSITIONS_PER_TURN = 6;

    private static final int FEEDBACK_DISPLAY_MS = 2000;
    private static final double TRIGGER_THRESHOLD = 0.5;

    // Distance-to-point target
    private static final double TARGET_X = 72;
    private static final double TARGET_Y = -144.0;

    private double distanceToTarget = 0.0;

    // ============================================================
    // DISTANCE -> VELOCITY BEST-FIT (Quadratic)
    // ============================================================
    private static final double VEL_A = 0.04541;
    private static final double VEL_B = -5.7004;
    private static final double VEL_C = 2146.31;

    private boolean autoShooterVelocity = true;

    // ============================================================
    // BUTTON HELPERS
    // ============================================================
    private final ButtonHelper btnCross = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnDpadUp = new ButtonHelper();
    private final ButtonHelper btnDpadDown = new ButtonHelper();

    private final ButtonHelper btnDpadLeft = new ButtonHelper();
    private final ButtonHelper btnDpadRight = new ButtonHelper();

    private final ButtonHelper btnSquare = new ButtonHelper();
    private final ButtonHelper btnL1 = new ButtonHelper();
    private final ButtonHelper btnR1 = new ButtonHelper();

    private final ButtonHelper btnL2 = new ButtonHelper();
    private final ButtonHelper btnR2 = new ButtonHelper();

    private final ButtonHelper btnOptions = new ButtonHelper();

    // ============================================================
    // CONSTRUCTOR
    // ============================================================
    public OperatorControls(Follower follower,
                            Intake intake,
                            SpinDex spinDex,
                            Shooter shooter,
                            Pusher pusher,
                            Telemetry telemetry,
                            ColorSensor colorSensor,
                            Limelight limelight) {

        this.follower = follower;
        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
        this.limelight = limelight;

        this.intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, shooter, telemetry);
        this.shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================
    public void update(Gamepad g2) {
        // Keep subsystem periodic updates here
        spinDex.periodic();

        Pose pose = follower.getPose();
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        distanceToTarget = Math.hypot(dx, dy);

        if (!autoAlignEnabled) {
            shooterVelocity = 0.0;
            shooter.setVelocity(0.0);
            shooterMode = ShooterMode.OFF;

            if (shooterMacro.isRunning()) {
                shooterMacro.stop();
            }
        }

        if (autoAlignEnabled && autoShooterVelocity && !shooterMacro.isRunning()) {
            shooterVelocity = velocityFromDistance(distanceToTarget);
        }

        intakeMacro.update();
        shooterMacro.update();

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

        if (!intakeMacro.isRunning()) {
            handleSpindexManual(g2);
        }

        if (!shooterMacro.isRunning() && autoAlignEnabled) {
            handleShooter(g2);
            handlePusher(g2);
            handleSmartAlign(g2);
        } else {
            if (shooterMacro.isRunning()) {
                shooterMode = ShooterMode.MACRO_RUNNING;
            }
        }

        pusher.update();
    }

    // ============================================================
    // DISTANCE -> VELOCITY FUNCTION
    // ============================================================
    private double velocityFromDistance(double d) {
        double v = (VEL_A * d * d) + (VEL_B * d) + VEL_C;

        if (v < 0.0) v = 0.0;
        if (v > SHOOTER_MAX_VELOCITY) v = SHOOTER_MAX_VELOCITY;

        return v;
    }

    // ============================================================
    // INTAKE (MACRO & MANUAL)
    // ============================================================
    private void handleIntake(Gamepad g2) {
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            if (intakeMacro.isRunning()) {
                intakeMacro.stop();
                intakeState = IntakeState.OFF;
                userFeedback = "Intake Macro Stopped";
            } else {
                intake.stop();
                intakeMacro.start();
                intakeState = IntakeState.MACRO_RUNNING;
                userFeedback = "Intake Macro Started";
            }
            feedbackTimer = System.currentTimeMillis();
        }

        if (intakeMacro.isRunning()) {
            return;
        }

        if (btnDpadDown.wasPressed(g2.dpad_down)) {
            spinDex.clearAllSlots();
            userFeedback = "SLOTS CLEARED";
            feedbackTimer = System.currentTimeMillis();
        }

        if (btnCircle.wasPressed(g2.circle)) {
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
    }

    // ============================================================
    // SMART ALIGNMENT
    // ============================================================
    private void handleSmartAlign(Gamepad g2) {
        if (shooterMacro.isRunning()) return;
        if (!autoAlignEnabled) return;

        if (btnCross.wasPressed(g2.cross)) {
            if (!spinDex.isEmpty()) {
                shooterMacro.start(shooterVelocity);
                userFeedback = "SHOOTER MACRO STARTED";
                feedbackTimer = System.currentTimeMillis();
                return;
            } else {
                userFeedback = "EMPTY! Cannot Start Shooter Macro";
                feedbackTimer = System.currentTimeMillis();
            }
        }

        if (btnL2.wasPressed(g2.left_trigger > TRIGGER_THRESHOLD)) {
            boolean found = spinDex.moveToGreenArtifact();
            if (found) {
                userFeedback = "Aligning: Purple";
            } else {
                userFeedback = "FAIL: No Purple Found";
            }
            feedbackTimer = System.currentTimeMillis();
        }

        if (btnR2.wasPressed(g2.right_trigger > TRIGGER_THRESHOLD)) {
            boolean found = spinDex.moveToPurpleArtifact();
            if (found) {
                userFeedback = "Aligning: Green";
            } else {
                userFeedback = "FAIL: No Green Found";
            }
            feedbackTimer = System.currentTimeMillis();
        }
    }

    // ============================================================
    // SHOOTER HANDLING
    // ============================================================
    private void handleShooter(Gamepad g2) {

        if (shooterMacro.isRunning()) return;

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
    // PUSHER
    // ============================================================
    private void handlePusher(Gamepad g2) {
        if (shooterMacro.isRunning()) return;
        if (!autoAlignEnabled) return;

        if (btnSquare.wasPressed(g2.square) && pusher.isReady()) {
            pusher.push();

            int currentPos = spinDex.getCurrentPosition();
            int posInTurn = currentPos % POSITIONS_PER_TURN;

            int firedSlot = -1;
            if (posInTurn == 3) firedSlot = 0;
            else if (posInTurn == 5) firedSlot = 1;
            else if (posInTurn == 1) firedSlot = 2;

            if (firedSlot != -1) {
                spinDex.clearSlot(firedSlot);
                userFeedback = "Manual Fire: Slot " + firedSlot;
                feedbackTimer = System.currentTimeMillis();
            }
        }
    }

    // ============================================================
    // MANUAL SPINDEX
    // ============================================================
    private void handleSpindexManual(Gamepad g2) {
        if (intakeMacro.isRunning() || shooterMacro.isRunning()) return;

        // Keep existing D-pad behavior
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(next);
        }

        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            int prev = spinDex.getCurrentPosition() - (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(prev);
        }

        // New micro adjust + rezero buttons
        if (btnR1.wasPressed(g2.right_bumper)) {
            spinDex.microAdjustRightAndRezero();
            userFeedback = "Spindex micro RIGHT + rezero";
            feedbackTimer = System.currentTimeMillis();
        }

        if (btnL1.wasPressed(g2.left_bumper)) {
            spinDex.microAdjustLeftAndRezero();
            userFeedback = "Spindex micro LEFT + rezero";
            feedbackTimer = System.currentTimeMillis();
        }
    }

    // ============================================================
    // TELEMETRY
    // ============================================================
    public void updateTelemetry() {
        if (limelight != null) {
            limelight.displayTelemetry();
        }

        telemetry.addData("Auto Align (Driver)", autoAlignEnabled ? "ENABLED" : "OFF");

        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Auto Velocity", autoShooterVelocity);

        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Actual Vel", "%.0f t/s", shooter.getAverageVelocity());

        telemetry.addData("Fit Vel (from dist)", "%.0f", velocityFromDistance(distanceToTarget));

        if (intakeMacro.isRunning()) {
            intakeMacro.addTelemetry();
        }
        if (shooterMacro.isRunning()) {
            shooterMacro.addTelemetry();
        }

        int currentPos = spinDex.getCurrentPosition();
        int turn = spinDex.getCurrentTurn();
        int posInTurn = currentPos % POSITIONS_PER_TURN;

        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Turn", turn);
        telemetry.addData("posInTurn", posInTurn);
        telemetry.addData("Motor Ticks", spinDex.getMotorPosition());

        telemetry.addData("Right Velocity", shooter.getRightVelocity());
        telemetry.addData("Left Velocity", shooter.getLeftVelocity());

        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        telemetry.addData("Position Tolerance", POSITION_TOLERANCE_TICKS);

        telemetry.addData("Target Point", "(%.0f, %.0f)", TARGET_X, TARGET_Y);
        telemetry.addData("Dist to Target", "%.2f", distanceToTarget);

        telemetry.addData("Is Pusher Ready", pusher.isReady());
    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        intakeMacro.stop();
        shooterMacro.stop();
    }
}