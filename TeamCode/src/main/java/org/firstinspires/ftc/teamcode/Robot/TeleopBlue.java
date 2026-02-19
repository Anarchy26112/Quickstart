package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    // Control handlers
    private DriverControlsBlue driverControlsBlue;
    private OperatorControls operatorControls;
    private LimelightTuning limelightTuning;

    // Subsystems
    private Intake intake;
    private SpinDex spin_dex;
    private Shooter shooter;
    private Pusher pusher;
    private ColorSensor colorSensor;
    private Limelight limelight;

    // Performance optimization
    private int loopCount = 0;
    private static final int TELEMETRY_UPDATE_FREQUENCY = 5;

    // Loop timing (nanoTime-based)
    private long lastLoopTimeNs = 0;
    private double lastLoopMs = 0;
    private double avgLoopMs = 0;
    private static final double LOOP_ALPHA = 0.1; // smoothing factor for EMA

    @Override
    public void init() {
        // 1. Initialize all subsystems
        intake = new Intake(hardwareMap, telemetry);
        spin_dex = new SpinDex(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        // 2. Initialize control handlers

        // Driver gets hardware map for Pedro Pathing
        driverControlsBlue = new DriverControlsBlue(hardwareMap, telemetry, limelight);

        // Operator gets subsystems to control them
        operatorControls = new OperatorControls(intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight, hardwareMap);
        limelightTuning = new LimelightTuning(intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Start loop timer baseline
        lastLoopTimeNs = System.nanoTime();

        // Keep your existing behavior
        pusher.push();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) {
            driverControlsBlue.startTeleopDrive();
        }
        telemetry.addData("Status", "Started");
        telemetry.update();

        // Reset loop timer at start to avoid huge first delta
        lastLoopTimeNs = System.nanoTime();
        lastLoopMs = 0;
        avgLoopMs = 0;
        loopCount = 0;
    }

    @Override
    public void loop() {
        // Measure time since last loop call using nanoTime
        long nowNs = System.nanoTime();
        if (lastLoopTimeNs != 0) {
            long deltaNs = nowNs - lastLoopTimeNs;
            lastLoopMs = deltaNs / 1_000_000.0;
        } else {
            lastLoopMs = 0;
        }
        lastLoopTimeNs = nowNs;

        // Exponential moving average (less jitter than raw per-loop)
        avgLoopMs = (avgLoopMs == 0)
                ? lastLoopMs
                : (LOOP_ALPHA * lastLoopMs + (1 - LOOP_ALPHA) * avgLoopMs);

        // Update both control systems
        if (driverControlsBlue != null) driverControlsBlue.update(gamepad1);
        if (operatorControls != null) operatorControls.update(gamepad1);
        operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        if (limelightTuning != null) limelightTuning.update(gamepad2);

        // Throttled telemetry updates
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {

            // Loop time telemetry
            telemetry.addData("Loop Time (ms)", "%.2f", lastLoopMs);
            telemetry.addData("Avg Loop (ms)", "%.2f", avgLoopMs);
            if (avgLoopMs > 0) telemetry.addData("Loop Rate (Hz)", "%.1f", 1000.0 / avgLoopMs);

            // Your existing telemetry
            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry();
            if (operatorControls != null) operatorControls.updateTelemetry();
            if (limelightTuning != null) limelightTuning.updateTelemetry();

            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // Stop all subsystems
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
