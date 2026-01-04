package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class DriverControlsBlue {

    private final Follower follower;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;
    private final Pose parkingBlue = new Pose(25, -36, 0);
    private PathChain parkingBluePath;
    private boolean homingMechanismEngaged = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();


    // --- Cached values so telemetry matches what we ACTUALLY applied ---
    private double lastVisionTurn = 0.0;      // Limelight turn output used this loop (pre slow-scaling)

    public DriverControlsBlue(HardwareMap hardwareMap, Telemetry telemetry, Limelight limelight) {
        this.telemetry = telemetry;
        this.limelight = limelight;

        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        // Update follower (required every loop)
        follower.update();

        // Update Limelight every loop so target visibility / tx/ty are fresh
        if (limelight != null) {
            limelight.update();
        }

        // Toggle auto-align with touchpad
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        // Fast mode while NOT holding left stick button
        slowMode = !gamepad1.left_stick_button;

        // Base drive inputs
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Default cached outputs
        lastVisionTurn = 0.0;
        if(btnCircle.wasPressed(gamepad1.circle)){
            homingMechanismEngaged = !homingMechanismEngaged;
        }
        if(homingMechanismEngaged){
            drive = 0.0;
            strafe = 0.0;
            turn = 0.0;
            followParkingPath();
        }

        // If auto-align is enabled and target is visible, override turn with Limelight
        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            lastVisionTurn = limelight.getTurnPowerSmartOffsetByDistance(
                    HamiltonParams.OFFSET_SWITCH_DISTANCE_IN,
                    HamiltonParams.TX_OFFSET_FAR_DEG_BLUE
            );
            turn = lastVisionTurn;
            //turn = getOdometryTurnPower();
        }
        if(btnPS.wasPressed(gamepad1.ps)) slowMode = !slowMode;
        // Apply to drivetrain (and cache EXACT applied values)
        if (!slowMode) {
            follower.setTeleOpDrive(drive, strafe, turn, true);
        } else {
            double scaledDrive = drive * NORMAL_SPEED;
            double scaledStrafe = strafe * NORMAL_SPEED;
            double scaledTurn = turn * 0.2;

            follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, true);
        }
    }

    public double calculateTargetHeading(){
        double x = 144.0-follower.getPose().getX();
        double y = 0.0-follower.getPose().getY();
        return Math.atan2(y, x);
    }
    public double getOdometryTurnPower(){
        double heading = follower.getPose().getHeading();
        double target = calculateTargetHeading();
        double error = target - heading;
        return error/6.5;
    }

    public void updateTelemetry() {
        telemetry.addData("Drive Mode", slowMode ? "Full Speed (100%)" : "Slow Speed (55%)");
        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");

        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag " + limelight.getDetectedTagId());
            telemetry.addData("Vision Turn (Raw)", "%.3f", lastVisionTurn);

            // Helpful aiming context
            telemetry.addData("tx", "%.2f", limelight.getTx());
            telemetry.addData("distance (in)", "%.2f", limelight.getHorizontalDistance());
        }

        telemetry.addData("X", String.format(Locale.US, "%.1f", follower.getPose().getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.1f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format(Locale.US, "%.1f°",
                Math.toDegrees(follower.getPose().getHeading())));
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }
    public void followParkingPath(){
        follower.followPath(parkingBluePath);
    }
    private void buildPaths() {
        Pose currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY());
        parkingBluePath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingBlue))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0.0)
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2).build();
    }
}
