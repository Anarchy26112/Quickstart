package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

@TeleOp(name = "TeleOp Blue (Dual Driver)")
public class TeleopBlueDual extends OpMode {

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
        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);

        operatorControls = new OperatorControls(
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                0,
                0
        );

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();

            if (restoredAutoPose != null) {
                double teleopX = restoredAutoPose.getY();
                double teleopY = -restoredAutoPose.getX();
                double teleopH = restoredAutoPose.getHeading() - Math.toRadians(90);

                restoredTeleopPose = new Pose(teleopX, teleopY, teleopH);
                follower.setPose(restoredTeleopPose);

                PoseHandoff.clear();

                telemetry.addData("Restored Pose (Auto)", "X=%.1f Y=%.1f H=%.1f°",
                        restoredAutoPose.getX(), restoredAutoPose.getY(), Math.toDegrees(restoredAutoPose.getHeading()));
                telemetry.addData("Restored Pose (TeleOp)", "X=%.1f Y=%.1f H=%.1f°",
                        restoredTeleopPose.getX(), restoredTeleopPose.getY(), Math.toDegrees(restoredTeleopPose.getHeading()));
            } else {
                telemetry.addData("Restored Pose", "Handoff present, but pose was null");
            }
        } else {
            telemetry.addData("Restored Pose", "NONE");
        }

        telemetry.addData("Status", "Initialized (Shared Follower, Dual Driver)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) driverControlsBlue.startTeleopDrive();
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        Pose pose = follower != null ? follower.getPose() : null;
        Vector vel = follower != null ? follower.getVelocity() : null;

        if (driverControlsBlue != null) driverControlsBlue.update(gamepad1, pose, nowMs);

        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        if (operatorControls != null) operatorControls.update(gamepad2, pose, vel, nowMs);
        if (limelightTuning != null) limelightTuning.update(gamepad2);

        if (follower != null) follower.update();

        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry(pose);
            if (operatorControls != null) operatorControls.updateTelemetry(nowMs);
            if (limelightTuning != null) limelightTuning.updateTelemetry();
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (operatorControls != null) operatorControls.stopAll();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}