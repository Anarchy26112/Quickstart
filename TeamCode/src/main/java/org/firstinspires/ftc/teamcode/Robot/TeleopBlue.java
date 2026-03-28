package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import com.pedropathing.follower.Follower;

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
    private static final int TELEMETRY_UPDATE_FREQUENCY = 15;

    private static final boolean TUNING_MODE = false;

    private Pose restoredAutoPose = null;
    private Pose restoredTeleopPose = null;

    private List<LynxModule> allHubs;
    private int hubCount;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        hubCount = allHubs.size();

        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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
                restoredTeleopPose = new Pose(
                        restoredAutoPose.getX(),
                        restoredAutoPose.getY(),
                        restoredAutoPose.getHeading()
                );
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
        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).clearBulkCache();
        }

        if (driverControlsBlue != null) {
            driverControlsBlue.update(gamepad1);
        }

        if (operatorControls != null && driverControlsBlue != null) {
            if (!TUNING_MODE) {
                operatorControls.update(gamepad2);
            }

            if (operatorControls.shouldEnableAutoAlign()) {
                driverControlsBlue.forceEnableAutoAlign();
                operatorControls.clearEnableAutoAlignRequest();
            }

            if (operatorControls.shouldDisableAutoAlign()) {
                driverControlsBlue.forceDisableAutoAlign();
                operatorControls.clearDisableAutoAlignRequest();
            }

            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }

        if (TUNING_MODE && limelightTuning != null) {
            limelightTuning.update(gamepad2);
        }

        if (shooter != null) {
            shooter.update();
        }

        if (follower != null) {
            follower.update();
        }

        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            telemetry.addData("Mode", TUNING_MODE ? "TUNING" : "COMPETITION");
            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry();
            if (!TUNING_MODE && operatorControls != null) operatorControls.updateTelemetry();
            if (TUNING_MODE && limelightTuning != null) limelightTuning.updateTelemetry();

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