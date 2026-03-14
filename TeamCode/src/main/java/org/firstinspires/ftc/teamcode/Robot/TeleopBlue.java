package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule; // <-- Added for Bulk Caching

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List; // <-- Added for Bulk Caching

@TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    private Follower follower;

    private DriverControlsBlue driverControlsBlue;
    private OperatorControls operatorControls;
    private LimelightTuning limelightTuning;

    private Intake intake;
    private Gate gate;
    private Shooter shooter;
    private Limelight limelight;

    private int loopCount = 0;
    private static final int TELEMETRY_UPDATE_FREQUENCY = 10;

    private Pose restoredAutoPose = null;
    private Pose restoredTeleopPose = null;

    // --- Hardware Bulk Caching ---
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        // --- 1. Initialize Bulk Caching (Highest Priority) ---
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Subsystems
        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        // ✅ ONE follower for the entire robot
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);
        operatorControls = new OperatorControls(follower, intake, shooter, telemetry, limelight, gate);

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        // Pose restore (apply to shared follower once)
        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                double teleopX = -restoredAutoPose.getY();
                double teleopY = restoredAutoPose.getX();
                double teleopH = restoredAutoPose.getHeading() - Math.toRadians(90);

                restoredTeleopPose = new Pose(teleopX, teleopY, teleopH);
                follower.setPose(restoredTeleopPose);

                PoseHandoff.clear();
            }
        }

        telemetry.addData("Status", "Initialized (Shared Follower + Bulk Caching)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) driverControlsBlue.startTeleopDrive();
    }

    @Override
    public void loop() {
        // --- 2. Clear the cache at the start of EVERY loop ---
        // This ensures all sensor/encoder reads in this loop pull fresh data from the single bulk read.
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (driverControlsBlue != null) driverControlsBlue.update(gamepad1);

        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        if (operatorControls != null) operatorControls.update(gamepad1);
        if (limelightTuning != null) limelightTuning.update(gamepad2);

        if (shooter != null) shooter.update();

        follower.update();

        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry();
            if (operatorControls != null) operatorControls.updateTelemetry();
            if (limelightTuning != null) limelightTuning.updateTelemetry();
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (operatorControls != null) operatorControls.stopAll();
    }
}