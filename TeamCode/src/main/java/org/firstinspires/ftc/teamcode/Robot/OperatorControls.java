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
    public static final int POSITIONS_PER_TURN = 6;
    private static final int FEEDBACK_DISPLAY_MS = 2000;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double Kd_INCREMENT = 0.01;
    private static final double KP_MIN = 0.0;
    private static final double KP_MAX = 0.1;

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
                            SpinDex spinDex,
                            Shooter shooter,
                            Pusher pusher,
                            Telemetry telemetry,
                            ColorSensor colorSensor,
                            Limelight limelight, HardwareMap hardwareMap) {

        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
        this.limelight = limelight;

        this.intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, shooter, telemetry);
        this.shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================

    public void update(Gamepad g2) {
        // CRITICAL: Update SpinDex periodic control first (runs PD controller)
        spinDex.periodic();

        follower.update();

        // Run all macros
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

        // Always handle intake controls (stop button needs to work anytime)
        handleIntake(g2);

        // We only block MANUAL SPINDEX while macro is running
        if (!intakeMacro.isRunning()) {
            handleSpindexManual(g2);
        }

        // Only allow shooting controls if the shooter macro isn't running
        if (!shooterMacro.isRunning()) {
            handleShooter(g2);
            handlePusher(g2);
            handleSmartAlign(g2);
        } else {
            shooterMode = ShooterMode.MACRO_RUNNING;
        }
        pusher.update();
    }



    // ============================================================
    // INTAKE (MACRO & MANUAL)
    // ============================================================

    private void handleIntake(Gamepad g2) {
        // 1. Start/Stop Toggle (Works anytime)
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            // Check the macro state directly for truth
            if (intakeMacro.isRunning()) {
                // STOP
                intakeMacro.stop();
                intakeState = IntakeState.OFF;
                userFeedback = "Intake Macro Stopped";
            } else {
                // START
                intake.stop();
                intakeMacro.start();
                intakeState = IntakeState.MACRO_RUNNING;
                userFeedback = "Intake Macro Started";
            }
            feedbackTimer = System.currentTimeMillis();
        }

        // ============================================================
        // GUARD: If macro is running, prevent other manual intake actions
        // ============================================================
        if (intakeMacro.isRunning()) {
            return;
        }

        // 2. Clear all slots (DPad Down)
        if (btnDpadDown.wasPressed(g2.dpad_down)) {
            spinDex.clearAllSlots();
            userFeedback = "SLOTS CLEARED";
            feedbackTimer = System.currentTimeMillis();
        }

        // 3. Toggle Manual Intake (Circle / B)
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
    // SMART ALIGNMENT (CROSS BUTTON)
    // ============================================================

    private void handleSmartAlign(Gamepad g2) {
        if (shooterMacro.isRunning()) return;

        // CROSS (A) = SHOOTER MACRO TRIGGER
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
        // LEFT TRIGGER (L2): Align to GREEN
        if (btnL2.wasPressed(g2.left_trigger > TRIGGER_THRESHOLD)) {
            boolean found = spinDex.moveToGreenArtifact();
            if (found) {
                userFeedback = "Aligning: Purple";
            } else {
                userFeedback = "FAIL: No Purple Found";
            }
            feedbackTimer = System.currentTimeMillis();
        }

        // RIGHT TRIGGER (R2): Align to PURPLE
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
    // SHOOTER HANDLING (MANUAL)
    // ============================================================

    private void handleShooter(Gamepad g2) {
        if (shooterMacro.isRunning()) return;

        boolean r1Pressed = btnR1.wasPressed(g2.right_bumper);
        boolean l1Pressed = btnL1.wasPressed(g2.left_bumper);
        boolean triangleHeld = g2.triangle;

        if (r1Pressed) {
            shooterVelocity = 2255.0;
        }
        else if (l1Pressed && triangleHeld) {
            shooterVelocity = 0.0;
        }
        else if (l1Pressed) {
            shooterVelocity = SHOOTER_MAX_VELOCITY * 0.73; //used to be 73%
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
    // PUSHER (MANUAL ONLY)
    // ============================================================

    private void handlePusher(Gamepad g2) {
        if (shooterMacro.isRunning()) return;

        if (btnSquare.wasPressed(g2.square) && pusher.isReady()) {
            pusher.push();

            int currentPos = spinDex.getCurrentPosition();
            int posInTurn = currentPos % POSITIONS_PER_TURN;

            // Position-to-Slot mapping with SHOOTING_OFFSET = 3:
            // Position 3 -> Slot 0
            // Position 5 -> Slot 1
            // Position 1 -> Slot 2
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
    // MANUAL SPINDEX (DIRECT POSITION CONTROL)
    // ============================================================

    private void handleSpindexManual(Gamepad g2) {
        if (intakeMacro.isRunning() || shooterMacro.isRunning()) return;

        // D-Pad Right: Move forward
        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(next);
        }

        // D-Pad Left: Move backward
        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            int prev = spinDex.getCurrentPosition() - (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(prev);
        }
        if (btnOptions.wasPressed(g2.options)){
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 2 : 3);
            spinDex.moveToPosition(next);
        }

    }

    // ============================================================
    // EMERGENCY STOP
    // ============================================================


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
        telemetry.addData("left Velocity", shooter.getLeftVelocity());

        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        telemetry.addData("P: ", follower.getPose().getX());

    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        intakeMacro.stop();
        shooterMacro.stop();
    }
}