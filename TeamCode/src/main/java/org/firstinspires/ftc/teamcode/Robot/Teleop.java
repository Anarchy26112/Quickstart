package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    private TelemetryManager telemetryM;
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
        // Initialize telemetry FIRST (required by subsystems)
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize all subsystems
        intake = new Intake(hardwareMap, telemetryM);
        spin_dex = new SpinDex(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        pusher = new Pusher(hardwareMap, telemetryM);

        // Initialize control handlers
        driverControls = new DriverControls(hardwareMap, telemetryM);
        operatorControls = new OperatorControls(intake, spin_dex, shooter, pusher, telemetryM);

        telemetryM.debug("Status: Initialized");
        telemetryM.update();
    }

    @Override
    public void start() {
        driverControls.startTeleopDrive();
        telemetryM.debug("Status: Started");
        telemetryM.update();
    }

    @Override
    public void loop() {
        // Update both control systems
        driverControls.update(gamepad1);
        operatorControls.update(gamepad2);

        // Throttled telemetry updates
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            driverControls.updateTelemetry(gamepad1);
            operatorControls.updateTelemetry();
            telemetryM.update();
        }
    }

    @Override
    public void stop() {
        // Stop all subsystems
        operatorControls.stopAll();

        telemetryM.debug("Status: Stopped");
        telemetryM.update();
    }
}