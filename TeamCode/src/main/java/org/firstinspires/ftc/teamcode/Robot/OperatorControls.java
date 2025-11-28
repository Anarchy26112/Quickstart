package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;

public class OperatorControls {

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

    // Feedback field for Smart Align failures (e.g., "No Purple Found")
    private String userFeedback = "";
    private long feedbackTimer = 0;

    // ============================================================
    // STATES
    // ============================================================

    private enum IntakeState { OFF, INTAKING, SPITTING, MACRO_RUNNING }
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
    private static final int POSITIONS_PER_TURN = 6;
    private static final int FEEDBACK_DISPLAY_MS = 2000;
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ============================================================
    // BUTTON HELPERS
    // ============================================================

    // Intake / Spindex Misc
    private final ButtonHelper btnCross = new ButtonHelper(); // SHOOTER MACRO TRIGGER
    private final ButtonHelper btnCircle = new ButtonHelper(); // Spit toggle
    private final ButtonHelper btnDpadUp = new ButtonHelper(); // Intake Macro
    private final ButtonHelper btnDpadDown = new ButtonHelper(); // Clear Slots

    // Manual Spindex
    private final ButtonHelper btnDpadLeft = new ButtonHelper();
    private final ButtonHelper btnDpadRight = new ButtonHelper();

    // Shooter / Pusher
    private final ButtonHelper btnSquare = new ButtonHelper(); // Fire pusher (manual only)
    private final ButtonHelper btnL1 = new ButtonHelper(); // Power -
    private final ButtonHelper btnR1 = new ButtonHelper(); // Power +

    // Smart Align Triggers
    private final ButtonHelper btnL2 = new ButtonHelper(); // Align Purple
    private final ButtonHelper btnR2 = new ButtonHelper(); // Align Green

    private final ButtonHelper btnShare = new ButtonHelper(); // E-Stop

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public OperatorControls(Intake intake,
                            SpinDex spinDex,
                            Shooter shooter,
                            Pusher pusher,
                            Telemetry telemetry,
                            ColorSensor colorSensor,
                            // 🌟 NEW: Add Limelight to constructor
                            Limelight limelight) {

        this.intake = intake;
        this.spinDex = spinDex;
        this.shooter = shooter;
        this.pusher = pusher;
        this.telemetry = telemetry;
        this.colorSensor = colorSensor;
        this.limelight = limelight; // 🌟 Initialize Limelight

        this.intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, telemetry);
        this.shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);
    }

    // ============================================================
    // UPDATE LOOP
    // ============================================================

    public void update(Gamepad g2) {
        // Run all macros first
        intakeMacro.update();
        shooterMacro.update();

        limelight.update();

        // Check if macros just finished
        if (intakeMacro.isComplete()) {
            intakeState = IntakeState.OFF;
        }

        if (shooterMacro.isComplete() || shooterMacro.hasFailed()) {
            shooterMode = ShooterMode.OFF; // Reset mode
            // Provide feedback if the macro failed due to empty magazine
            if (shooterMacro.hasFailed()) {
                userFeedback = "SHOOTER MACRO FAILED: Empty Slots";
                feedbackTimer = System.currentTimeMillis();
            }
        }

        // Handle controls only if the corresponding macro isn't running
        if (!intakeMacro.isRunning()) {
            handleIntake(g2);
            handleSpindexManual(g2);
        }

        // Only allow shooting controls if the shooter macro isn't running
        if (!shooterMacro.isRunning()) {
            handleShooter(g2);
            handlePusher(g2);
            handleSmartAlign(g2);
        } else {
            // While shooter macro is running, ensure shooter mode reflects it
            shooterMode = ShooterMode.MACRO_RUNNING;
        }

        handleEmergencyStop(g2);
        pusher.update();
    }

    // ============================================================
    // INTAKE (MACRO & SPIT ONLY)
    // ============================================================

    private void handleIntake(Gamepad g2) {
        // 1. Start Intake Macro (DPad Up)
        if (btnDpadUp.wasPressed(g2.dpad_up)) {
            if (intakeState != IntakeState.MACRO_RUNNING) {
                intake.stop();
                intakeMacro.start();
                intakeState = IntakeState.MACRO_RUNNING;
                userFeedback = "Intake Macro Started";
                feedbackTimer = System.currentTimeMillis();
            } else {
                intakeMacro.stop();
                intakeState = IntakeState.OFF;
                userFeedback = "Intake Macro Stopped";
                feedbackTimer = System.currentTimeMillis();
            }
        }

        // 2. Clear all slots (DPad Down) - In case tracking gets messed up
        if (btnDpadDown.wasPressed(g2.dpad_down)) {
            spinDex.clearAllSlots();
            userFeedback = "SLOTS CLEARED";
            feedbackTimer = System.currentTimeMillis();
        }

        // 3. Toggle Spit (Circle / B)
        if (btnCircle.wasPressed(g2.circle)) {
            switch (intakeState) {
                case SPITTING:
                    intake.stop();
                    intakeState = IntakeState.OFF;
                    break;
                default:
                    intake.spit();
                    intakeState = IntakeState.SPITTING;
                    break;
            }
        }
    }

    // ============================================================
    // SMART ALIGNMENT (CROSS & TRIGGERS)
    // ============================================================

    private void handleSmartAlign(Gamepad g2) {
        if (shooterMacro.isRunning()) return;

        // CROSS (A) = SHOOTER MACRO TRIGGER
        if (btnCross.wasPressed(g2.cross)) {
            // Only start if the macro isn't already running and the magazine isn't empty
            if (!spinDex.isEmpty()) {
                // Start the macro at the current set shooterVelocity
                shooterMacro.start(shooterVelocity);
                userFeedback = "SHOOTER MACRO STARTED";
                feedbackTimer = System.currentTimeMillis();
                return; // Exit the function to prevent other actions
            } else {
                userFeedback = "EMPTY! Cannot Start Shooter Macro";
                feedbackTimer = System.currentTimeMillis();
            }
        }

        // LEFT TRIGGER (L2): Align to PURPLE
        if (btnL2.wasPressed(g2.left_trigger > TRIGGER_THRESHOLD)) {
            boolean found = spinDex.moveToPurpleArtifact();
            if (found) {
                userFeedback = "Aligning: Purple";
            } else {
                userFeedback = "FAIL: No Purple Found";
            }
            feedbackTimer = System.currentTimeMillis();
        }

        // RIGHT TRIGGER (R2): Align to GREEN
        if (btnR2.wasPressed(g2.right_trigger > TRIGGER_THRESHOLD)) {
            boolean found = spinDex.moveToGreenArtifact();
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
        // Block manual control if the shooter macro is running
        if (shooterMacro.isRunning()) return;

        boolean r1Pressed = btnR1.wasPressed(g2.right_bumper);
        boolean l1Pressed = btnL1.wasPressed(g2.left_bumper);
        boolean triangleHeld = g2.triangle;

        // Presets with Triangle modifier
        if (r1Pressed && triangleHeld) { // R1 + Triangle: 80% Power
            shooterVelocity = SHOOTER_MAX_VELOCITY * 0.80;
        }
        else if (l1Pressed && triangleHeld) { // L1 + Triangle: Stop
            shooterVelocity = 0.0;
        }
        // Manual Increment
        else if (r1Pressed) { // R1 only: Increment
            shooterVelocity = Math.min(SHOOTER_MAX_VELOCITY, shooterVelocity + VELOCITY_INCREMENT);
        }
        else if (l1Pressed) { // L1 only: Decrement
            shooterVelocity = Math.max(0.0, shooterVelocity - VELOCITY_INCREMENT);
        }

        shooter.setVelocity(shooterVelocity);

        // Determine shooter mode based on actual velocity
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
    // PUSHER (MANUAL ONLY - Macro handles its own slot clearing)
    // ============================================================

    private void handlePusher(Gamepad g2) {
        // Block manual push if the shooter macro is running
        if (shooterMacro.isRunning()) return;

        if (btnSquare.wasPressed(g2.square) && pusher.isReady()) {
            pusher.push();

            // Clear the slot that was just fired manually
            int currentPos = spinDex.getCurrentPosition();
            int posInTurn = currentPos % POSITIONS_PER_TURN;

            // Shooting positions: 1->Slot0, 3->Slot1, 5->Slot2
            int firedSlot = -1;
            if (posInTurn == 1) firedSlot = 0;
            else if (posInTurn == 3) firedSlot = 1;
            else if (posInTurn == 5) firedSlot = 2;

            if (firedSlot != -1) {
                spinDex.clearSlot(firedSlot);
                userFeedback = "Manual Fire: Slot " + firedSlot;
                feedbackTimer = System.currentTimeMillis();
            }
        }
    }

    // ============================================================
    // MANUAL SPINDEX (OVERRIDES)
    // ============================================================

    private void handleSpindexManual(Gamepad g2) {
        if (intakeMacro.isRunning() || shooterMacro.isRunning()) return;

        if (btnDpadRight.wasPressed(g2.dpad_right)) {
            int next = spinDex.getCurrentPosition() + (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(next);
        }

        if (btnDpadLeft.wasPressed(g2.dpad_left)) {
            int prev = spinDex.getCurrentPosition() - (g2.triangle ? 1 : 2);
            spinDex.moveToPosition(prev);
        }
    }

    // ============================================================
    // EMERGENCY STOP
    // ============================================================

    private void handleEmergencyStop(Gamepad g2) {
        if (btnShare.wasPressed(g2.share)) {
            stopAll();
            intakeMacro.stop();
            shooterMacro.stop();
            spinDex.clearAllSlots();
            intakeState = IntakeState.OFF;
            shooterMode = ShooterMode.OFF;

            // Reset inputs
            btnCross.reset(); btnCircle.reset(); btnSquare.reset();
            btnL1.reset(); btnR1.reset();
            btnDpadLeft.reset(); btnDpadRight.reset();
            btnDpadUp.reset(); btnDpadDown.reset();
            btnL2.reset(); btnR2.reset();

            userFeedback = "E-STOP ACTIVATED. ALL SYSTEMS OFF.";
            feedbackTimer = System.currentTimeMillis();
            telemetry.update();
        }
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    public void updateTelemetry() {

        // Display Limelight telemetry first
        limelight.displayTelemetry();

        // Show feedback for 2 seconds
        if (System.currentTimeMillis() - feedbackTimer < FEEDBACK_DISPLAY_MS) {
            telemetry.addData("ACTION", userFeedback);
        }

        // Shooter
        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Target Vel", "%.0f t/s", shooterVelocity);
        telemetry.addData("Actual Vel", "%.0f t/s", shooter.getAverageVelocity());

        // Macro status
        if (intakeMacro.isRunning()) {
            intakeMacro.addTelemetry();
        }
        if (shooterMacro.isRunning()) {
            shooterMacro.addTelemetry();
        }

        // Spindex - now shows turn and position within turn
        int currentPos = spinDex.getCurrentPosition();
        int turn = spinDex.getCurrentTurn();
        int posInTurn = currentPos % POSITIONS_PER_TURN;

        telemetry.addData("Spindex Pos", "%d (Turn %d, Pos %d)", currentPos, turn, posInTurn);
        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        // Color Sensors
        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

    }

    public void stopAll() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        intakeMacro.stop();
        shooterMacro.stop();
    }
}