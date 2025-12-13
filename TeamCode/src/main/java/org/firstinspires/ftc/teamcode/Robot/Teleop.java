package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp (2 Drivers)")
public class Teleop extends OpMode {

    // Control handlers
    private DriverControls driverControls;
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
        driverControls = new DriverControls(hardwareMap, telemetry, limelight);

        // Operator gets subsystems to control them
        operatorControls = new OperatorControls(intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight);
        limelightTuning = new LimelightTuning(intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight);

        //operatorControls.initializePusher();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControls != null) {
            driverControls.startTeleopDrive();
        }
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update both control systems
        if (driverControls != null) driverControls.update(gamepad1);
        if (operatorControls != null) operatorControls.update(gamepad2);
        if (limelightTuning != null) limelightTuning.update(gamepad2);


        // Throttled telemetry updates
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControls != null) driverControls.updateTelemetry();
            if (operatorControls != null) operatorControls.updateTelemetry();
            if (limelightTuning != null) limelightTuning.updateTelemetry();
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