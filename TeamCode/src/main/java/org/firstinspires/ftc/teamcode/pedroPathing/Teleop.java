package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

        // Initialize control handlers
        driverControls = new DriverControls(hardwareMap, telemetryM);
        operatorControls = new OperatorControls(intake, spin_dex, shooter, telemetryM);

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