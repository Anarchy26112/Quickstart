package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

@TeleOp(name = "TeleOp Blue (Dual Driver)")
public class TeleopBlueDual extends OpMode {

    // ✅ ONE follower shared by the whole robot
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

        // ✅ Create follower ONCE here
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // ✅ Pass SAME follower to both controllers
        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);

        // NOTE: this assumes your OperatorControls constructor matches the newer shared-follower version:
        // OperatorControls(Follower follower, Intake intake, Shooter shooter, Telemetry telemetry, Limelight limelight, Gate gate)
        operatorControls = new OperatorControls(
                follower,
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                0,0
        );
        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        // ---- Pose restore (Auto -> TeleOp frame) ----
        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();

            if (restoredAutoPose != null) {
                double teleopX = restoredAutoPose.getY();
                double teleopY = -restoredAutoPose.getX();
                double teleopH = restoredAutoPose.getHeading() - Math.toRadians(90);

                restoredTeleopPose = new Pose(teleopX, teleopY, teleopH);

                // ✅ Apply to shared follower once
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
        // Driver on gamepad1
        if (driverControlsBlue != null) driverControlsBlue.update(gamepad1);

        // ✅ Bridge driver auto-align toggle into operator logic (shooter gating, auto velocity, etc.)
        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        // Operator + tuning on gamepad2
        if (operatorControls != null) operatorControls.update(gamepad2);
        if (limelightTuning != null) limelightTuning.update(gamepad2);

        // ✅ follower update once per loop
        if (follower != null) follower.update();

        // Telemetry throttled
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
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}