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

public class DriverControlsRed {

    private final Follower follower;
    private final Telemetry telemetry;
    private final Limelight limelight;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();

    private final Pose parkingRed = new Pose(25, 30, 0);
    private PathChain parkingRedPath;
    private boolean homingMechanismEngaged = false;

    // --- Cached values so telemetry matches what we ACTUALLY applied ---
    private double lastVisionTurn = 0.0;      // Limelight turn output used this loop (pre scaling)
    private double lastAppliedTurn = 0.0;     // Final turn sent to follower (post scaling)
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    public DriverControlsRed(HardwareMap hardwareMap, Telemetry telemetry, Limelight limelight) {
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
            limelight.setTargetRed();
        }

        // Toggle auto-align with touchpad
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        // Toggle slow mode with PS button
        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        // Base drive inputs
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Cache base inputs for telemetry
        lastDrive = drive;
        lastStrafe = strafe;

        // Default cached outputs
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;

        // Toggle homing / parking path
        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;
        }

        // If homing, kill manual inputs and follow path
        if (homingMechanismEngaged) {
            drive = 0.0;
            strafe = 0.0;
            turn = 0.0;
            followParkingPath();
        }

        boolean usingVisionTurn = false;

        if (!homingMechanismEngaged && autoAlignEnabled && limelight != null) {

            if (limelight.isTargetVisible()) {
                usingVisionTurn = true;

            }
        }


        // --- Scaling rules you requested ---
        // Normal mode: translation 100%, rotation 67%
        // Slow mode: translation NORMAL_SPEED, rotation 25%
        // If Limelight is providing turn: rotation 100% (regardless of slowMode)
        double translationScale = slowMode ? NORMAL_SPEED : 1.0;

        double rotationScale;
        if (usingVisionTurn) {
            rotationScale = 1.0;
        } else {
            rotationScale = slowMode ? 0.25 : 0.67;
        }

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;
        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, true);
    }

    public double calculateTargetHeading() {
        double x = 144.0 - follower.getPose().getX();
        double y = -72.0 - follower.getPose().getY();
        return Math.atan2(y, x);
    }

    public double getOdometryTurnPower() {
        double heading = follower.getPose().getHeading();
        double target = calculateTargetHeading();
        double error = target - heading;
        return error / 6.5;
    }

    public void updateTelemetry() {
        telemetry.addData("Drive Mode", slowMode ? "Slow Speed (55%)" : "Full Speed (100%)");
        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");

        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag " + limelight.getDetectedTagId());
            telemetry.addData("Vision Turn (Raw)", "%.3f", lastVisionTurn);

            telemetry.addData("tx", "%.2f", limelight.getTx());
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

    public void followParkingPath() {
        buildPaths();
        if (homingMechanismEngaged) follower.followPath(parkingRedPath);
    }

    private void buildPaths() {
        Pose currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY());
        parkingRedPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingRed))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0.0)
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();
    }
}
