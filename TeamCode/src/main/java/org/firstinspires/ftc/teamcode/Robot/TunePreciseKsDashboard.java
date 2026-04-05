package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NORMAL_SPEED;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.PRECISE_KD_TURN;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.PRECISE_KP_TURN;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.PRECISE_kS_VOLTAGE_COMP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Tune Precise kS Dashboard", group = "Tests")
public class TunePreciseKsDashboard extends LinearOpMode {

    public static double KS_STEP_SMALL = 0.001;
    public static double KS_STEP_BIG = 0.005;
    public static double MANUAL_TURN_SCALE = 0.25;

    private final ButtonHelper dpadUpButton = new ButtonHelper();
    private final ButtonHelper dpadDownButton = new ButtonHelper();
    private final ButtonHelper xButton = new ButtonHelper();
    private final ButtonHelper bButton = new ButtonHelper();
    private final ButtonHelper aButton = new ButtonHelper();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.createFollower(hardwareMap);
        follower.update();

        Limelight limelight = new Limelight(hardwareMap, telemetry);

        // Optional known pose
        // follower.setPose(new Pose(0, 0, 0));

        // Force PRECISE profile
        limelight.setRobotY(0);

        // Pick alliance target
        limelight.setTargetBlue();

        // For clean kS tuning, start with P and D at zero
        HamiltonParams.PRECISE_KP_TURN = 0.0;
        HamiltonParams.PRECISE_KD_TURN = 0.0;
        HamiltonParams.PRECISE_kS_VOLTAGE_COMP = 0.030;

        boolean autoAimEnabled = false;

        double lastHeadingDeg = 0.0;
        long lastTimeNanos = 0;

        telemetry.addLine("Ready for PRECISE kS tuning");
        telemetry.addLine("D-pad Up/Down = +/- small");
        telemetry.addLine("X/B = +/- big");
        telemetry.addLine("A = toggle auto aim");
        telemetry.addLine("Watch heading + headingVelocity on Dashboard");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        lastHeadingDeg = getHeadingDegrees(follower);
        lastTimeNanos = System.nanoTime();

        while (opModeIsActive()) {
            limelight.setRobotY(0);
            limelight.refreshTunables();

            if (dpadUpButton.wasPressed(gamepad1.dpad_up)) {
                HamiltonParams.PRECISE_kS_VOLTAGE_COMP += KS_STEP_SMALL;
            }
            if (dpadDownButton.wasPressed(gamepad1.dpad_down)) {
                HamiltonParams.PRECISE_kS_VOLTAGE_COMP -= KS_STEP_SMALL;
            }
            if (xButton.wasPressed(gamepad1.x)) {
                HamiltonParams.PRECISE_kS_VOLTAGE_COMP += KS_STEP_BIG;
            }
            if (bButton.wasPressed(gamepad1.b)) {
                HamiltonParams.PRECISE_kS_VOLTAGE_COMP -= KS_STEP_BIG;
            }
            if (aButton.wasPressed(gamepad1.a)) {
                autoAimEnabled = !autoAimEnabled;
            }

            HamiltonParams.PRECISE_kS_VOLTAGE_COMP =
                    Math.max(0.0, HamiltonParams.PRECISE_kS_VOLTAGE_COMP);

            double y = -gamepad1.left_stick_y * NORMAL_SPEED;
            double xStrafe = gamepad1.left_stick_x * NORMAL_SPEED;

            double turn;
            if (autoAimEnabled) {
                limelight.setTargetAngle(0.0);
                limelight.update(); // swap to updateControl() if that's the correct method in your class
                turn = limelight.getTurnPower();
            } else {
                limelight.pollVision();
                turn = gamepad1.right_stick_x * MANUAL_TURN_SCALE;
            }

            follower.setTeleOpDrive(y, xStrafe, turn, false);
            follower.update();

            double headingDeg = getHeadingDegrees(follower);

            long nowNanos = System.nanoTime();
            double dt = (nowNanos - lastTimeNanos) / 1e9;
            double headingVelocityDegPerSec = 0.0;

            if (dt > 1e-6) {
                headingVelocityDegPerSec = angleWrapDeg(headingDeg - lastHeadingDeg) / dt;
            }

            lastHeadingDeg = headingDeg;
            lastTimeNanos = nowNanos;

            Pose pose = follower.getPose();

            telemetry.addLine("=== PRECISE kS TUNING ===");
            telemetry.addData("Auto Aim", autoAimEnabled);
            telemetry.addData("Profile", limelight.getAimProfileName());
            telemetry.addData("PRECISE_KP", PRECISE_KP_TURN);
            telemetry.addData("PRECISE_KD", PRECISE_KD_TURN);
            telemetry.addData("PRECISE_kS", PRECISE_kS_VOLTAGE_COMP);

            telemetry.addData("Target Visible", limelight.isTargetVisible());
            telemetry.addData("tx", limelight.getTx());
            telemetry.addData("Error", limelight.getLastError());
            telemetry.addData("Turn Power", turn);

            telemetry.addData("Heading (deg)", headingDeg);
            telemetry.addData("Heading Vel (deg/s)", headingVelocityDegPerSec);

            if (pose != null) {
                telemetry.addData("Pose X", pose.getX());
                telemetry.addData("Pose Y", pose.getY());
            }

            telemetry.addData("Settled", limelight.isSettled());
            telemetry.addData("Shoot Ready", limelight.isShootReady());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("headingDeg", headingDeg);
            packet.put("headingVelocityDegPerSec", headingVelocityDegPerSec);
            packet.put("turnPower", turn);
            packet.put("limelightErrorDeg", limelight.getLastError());
            packet.put("precise_kS", HamiltonParams.PRECISE_kS_VOLTAGE_COMP);
            packet.put("targetVisible", limelight.isTargetVisible() ? 1 : 0);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private double getHeadingDegrees(Follower follower) {
        Pose pose = follower.getPose();
        if (pose == null) return 0.0;
        return Math.toDegrees(pose.getHeading());
    }

    private double angleWrapDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}