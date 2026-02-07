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

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnCircle = new ButtonHelper();

    private final Pose parkingBlue = new Pose(25, 30, 0);
    private PathChain parkingBluePath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    // --- State variables for Turning ---
    private boolean headingTurnEngaged = false;

    // --- Cached values for Telemetry ---
    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    public DriverControlsBlue(HardwareMap hardwareMap, Telemetry telemetry, Limelight limelight) {
        this.telemetry = telemetry;
        this.limelight = limelight;

        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.startTeleopDrive();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        // 1. Update Follower & Limelight (Essential)
        follower.update();
        if (limelight != null) {
            limelight.update();
            limelight.setTargetBlue();
        }

        // 2. Handle Inputs (Slow Mode / Auto Align Toggles)
        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
            // If we disable auto-align while holding a turn, release immediately
            if (!autoAlignEnabled && headingTurnEngaged) {
                headingTurnEngaged = false;
                startTeleopDrive();
            }
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        // 3. Homing / Parking Logic (Highest Priority)
        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;
            if (homingMechanismEngaged) {
                buildParkingPathOnce();
                follower.followPath(parkingBluePath);
                homingPathStarted = true;
                headingTurnEngaged = false; // Override any auto-align
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            return; // Exit early, let path follower run
        }

        // 4. Auto-Align Logic
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        boolean usingVisionTurn = false;

        lastDrive = drive;
        lastStrafe = strafe;

        if (autoAlignEnabled) {
            boolean targetVisible = (limelight != null && limelight.isTargetVisible());

            if (targetVisible) {
                // CASE A: Target Visible -> Use Vision

                // If we were previously blind-turning (holding point), release it now
                if (headingTurnEngaged) {
                    headingTurnEngaged = false;
                    startTeleopDrive();
                }

                lastVisionTurn = limelight.getTurnPowerSmartOffsetByDistance(
                        OFFSET_SWITCH_DISTANCE_IN,
                        TX_OFFSET_FAR_DEG_BLUE
                );
                turn = lastVisionTurn;
                usingVisionTurn = true;
                lastTurnSource = "VISION";

            } else {
                // CASE B: Target NOT Visible -> Snap Turn (Blind)

                // If we aren't holding yet, lock position and snap heading ONCE
                if (!headingTurnEngaged) {
                    double targetHeading = calculateTargetHeading();

                    // Lock current X/Y, snap to Target Heading
                    Pose lockPose = new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            targetHeading
                    );

                    follower.holdPoint(lockPose);
                    headingTurnEngaged = true;
                }

                lastTurnSource = "HOLD_POINT";
                return; // Exit here. Follower handles the hold. Do not send TeleOp drive commands.
            }
        } else {
            // Auto Align is OFF - Standard Manual Drive
            lastTurnSource = "MANUAL";

            // Safety: If we somehow got here while flag is true, reset
            if (headingTurnEngaged) {
                headingTurnEngaged = false;
                startTeleopDrive();
            }
        }

        // 5. Apply Final Drive Powers
        applyScaledDrive(drive, strafe, turn, usingVisionTurn);
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingVisionTurn) {
        double translationScale = slowMode ? NORMAL_SPEED : 1.0;

        // If vision is active, give full rotation power to the PID; otherwise scale for driver
        double rotationScale = usingVisionTurn ? 1.0 : (slowMode ? 0.20 : 0.5);

        lastTranslationScale = translationScale;
        lastRotationScale = rotationScale;

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, true);
    }

    public double calculateTargetHeading() {
        // If X > 36, face 45 degrees, else face 22.5 degrees
        double targetDeg = (follower.getPose().getX() > 36) ? 45.0 : 22.5;
        return Math.toRadians(targetDeg);
    }

    public void updateTelemetry() {
        telemetry.addData("Drive Mode", slowMode
                ? String.format(Locale.US, "Slow (%.0f%%)", NORMAL_SPEED * 100.0)
                : "Full (100%)");

        telemetry.addData("Auto-Align", autoAlignEnabled ? "ENABLED" : "OFF");
        telemetry.addData("State", homingMechanismEngaged ? "HOMING" : (headingTurnEngaged ? "HOLDING ANGLE" : "TELEOP"));

        telemetry.addData("Turn Source", lastTurnSource);
        telemetry.addData("Vision Turn (Raw)", "%.3f", lastVisionTurn);
        telemetry.addData("Turn (Applied)", "%.3f", lastAppliedTurn);

        if (autoAlignEnabled && limelight != null && limelight.isTargetVisible()) {
            telemetry.addData("Aligning To", "Tag " + limelight.getDetectedTagId());
            telemetry.addData("tx", "%.2f", limelight.getTx());
        }

        telemetry.addData("Pose", String.format(Locale.US, "X:%.1f Y:%.1f H:%.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    private void buildParkingPathOnce() {
        Pose p = follower.getPose();
        Pose currentPose = new Pose(p.getX(), p.getY(), p.getHeading());

        parkingBluePath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, parkingBlue))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0.0)
                .setVelocityConstraint(0.025) // Very slow approach
                .setBrakingStrength(2) // Strong braking
                .build();
    }
}