package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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

@TeleOp(name = "TeleOp Red")
public class TeleopRed extends OpMode {

    private static final double RED_TARGET_X = 72;
    private static final double RED_TARGET_Y = 144.0;

    private Follower follower;

    private DriverControlsRed driverControlsRed;
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

        driverControlsRed = new DriverControlsRed(follower, telemetry, limelight);
        operatorControls = new OperatorControls(
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                RED_TARGET_X,
                RED_TARGET_Y
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
        if (driverControlsRed != null) driverControlsRed.startTeleopDrive();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        long nowMs = System.currentTimeMillis();
        Pose pose = follower != null ? follower.getPose() : null;
        Vector vel = follower != null ? follower.getVelocity() : null;

        if (driverControlsRed != null) driverControlsRed.update(gamepad1, pose, nowMs);

        if (operatorControls != null && driverControlsRed != null) {
            operatorControls.setAutoAlignEnabled(driverControlsRed.isAutoAlignEnabled());
        }

        if (operatorControls != null) operatorControls.update(gamepad1, pose, vel, nowMs);
        if (limelightTuning != null) limelightTuning.update(gamepad2);

        if (shooter != null) shooter.update();

        follower.update();

        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControlsRed != null) driverControlsRed.updateTelemetry(pose);
            if (operatorControls != null) operatorControls.updateTelemetry(nowMs);
            if (limelightTuning != null) limelightTuning.updateTelemetry();
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (operatorControls != null) operatorControls.stopAll();
    }
}