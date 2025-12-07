package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Red")
public class TeleOpRed extends OpMode {

    // Control handlers
    private DriverControlsRed driverControlsRed;
    private OperatorControls operatorControls;

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

    @Override
    public void init() {
        // 1. Initialize all subsystems
        intake = new Intake(hardwareMap, telemetry);
        spin_dex = new SpinDex(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);
        pusher.stop();
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);


        // 2. Initialize control handlers
        // IMPORTANT: Pass limelight to DriverControls
        driverControlsRed = new DriverControlsRed(hardwareMap, telemetry, limelight);

        // Operator gets subsystems to control them
        operatorControls = new OperatorControls(intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsRed != null) {
            driverControlsRed.startTeleopDrive();
        }
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (limelight != null) {
            limelight.update();
        }

        // Update both control systems
        if (driverControlsRed != null) driverControlsRed.update(gamepad1);
        if (operatorControls != null) operatorControls.update(gamepad1);

        // Throttled telemetry updates
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControlsRed != null) driverControlsRed.updateTelemetry();
            if (operatorControls != null) operatorControls.updateTelemetry();
        }
    }

    @Override
    public void stop() {
        // Stop all subsystems
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
        if (limelight != null) {
            limelight.stop();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}