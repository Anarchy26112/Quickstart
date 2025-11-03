package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp")
public class Teleop extends OpMode {

    // Control handlers
    private DriverControls driverControls;
    private OperatorControls operatorControls;
    private Follower follower;

    // Subsystems
    private Intake intake;
    private SpinDex spin_dex;
    private Shooter shooter;
    private Pusher pusher;

    // Performance optimization
    private int loopCount = 0;
    private static final int TELEMETRY_UPDATE_FREQUENCY = 5; // Update every 5 loops (~100ms)

    @Override
    public void init() {
        // Initialize all subsystems (pass standard telemetry)
        intake = new Intake(hardwareMap, telemetry);
        spin_dex = new SpinDex(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);

        // Initialize control handlers
        driverControls = new DriverControls(hardwareMap, telemetry);
        operatorControls = new OperatorControls(intake, spin_dex, shooter, pusher, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // driverControls.startTeleopDrive();
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update both control systems
        // driverControls.update(gamepad1);
        operatorControls.update(gamepad2);

        // Throttled telemetry updates
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            // driverControls.updateTelemetry(gamepad1);
            operatorControls.updateTelemetry();
        }
    }

    @Override
    public void stop() {
        // Stop all subsystems
        operatorControls.stopAll();

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}