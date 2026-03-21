package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    private static final double BLUE_TARGET_X = 72;
    private static final double BLUE_TARGET_Y = -144.0;

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

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);
        operatorControls = new OperatorControls(
                follower,
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                BLUE_TARGET_X,
                BLUE_TARGET_Y
        );

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                double teleopX = restoredAutoPose.getX();
                double teleopY = restoredAutoPose.getY();
                double teleopH = restoredAutoPose.getHeading();

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
        if (driverControlsBlue != null) {
            driverControlsBlue.startTeleopDrive();
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (driverControlsBlue != null) {
            driverControlsBlue.update(gamepad1);
        }

        // Push driver-owned auto-align state into operator controls
        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        // Operator should be on gamepad2
        if (operatorControls != null) {
            operatorControls.update(gamepad2);
        }

        // If shooting finished, operator requests auto-align disable.
        // Disable it in the driver class too, so it does not get re-enabled next loop.
        if (operatorControls != null
                && driverControlsBlue != null
                && operatorControls.shouldDisableAutoAlign()) {

            driverControlsBlue.forceDisableAutoAlign();
            operatorControls.setAutoAlignEnabled(false);
            operatorControls.clearDisableAutoAlignRequest();
        }

        if (limelightTuning != null) {
            limelightTuning.update(gamepad2);
        }

        if (shooter != null) {
            shooter.update();
        }

        follower.update();

        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControlsBlue != null) {
                driverControlsBlue.updateTelemetry();
            }
            if (operatorControls != null) {
                operatorControls.updateTelemetry();
            }
            if (limelightTuning != null) {
                limelightTuning.updateTelemetry();
            }
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
    }
}