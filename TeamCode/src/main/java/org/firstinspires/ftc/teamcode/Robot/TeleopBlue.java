package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "TeleOp Blue (Dual Driver)")
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
    private static final int TELEMETRY_UPDATE_FREQUENCY = 5;

    private Pose restoredAutoPose = null;
    private Pose restoredTeleopPose = null;

    @Override
    public void init() {
        // Subsystems
        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        // ✅ ONE follower for the entire robot
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // ✅ Pass the SAME follower to both controllers
        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);
        operatorControls = new OperatorControls(follower, intake, shooter, telemetry, limelight, gate);

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        // Pose restore (apply to shared follower once)
        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                double teleopX = restoredAutoPose.getY();
                double teleopY = -restoredAutoPose.getX();
                double teleopH = restoredAutoPose.getHeading() - Math.toRadians(90);

                restoredTeleopPose = new Pose(teleopX, teleopY, teleopH);
                follower.setPose(restoredTeleopPose);

                PoseHandoff.clear();
            }
        }

        telemetry.addData("Status", "Initialized (Shared Follower)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) driverControlsBlue.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (driverControlsBlue != null) driverControlsBlue.update(gamepad1);

        // ✅ Bridge the driver's auto-align toggle into operator logic
        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        if (operatorControls != null) operatorControls.update(gamepad1);
        if (limelightTuning != null) limelightTuning.update(gamepad2);

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