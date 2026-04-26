package org.firstinspires.ftc.teamcode.Robot.Controls;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.ButtonHelper;
import org.firstinspires.ftc.teamcode.Robot.GoalAimController;

public class DriverControlsRed {

    private final Follower follower;
    private final Telemetry telemetry;
    private final GoalAimController aimController;

    private boolean slowMode = false;
    private boolean autoAlignEnabled = false;
    private boolean intakingActive = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();
    private final ButtonHelper btnTriangle = new ButtonHelper();
    private final ButtonHelper btnCross = new ButtonHelper();

    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    public DriverControlsRed(Follower follower, Telemetry telemetry, GoalAimController aimController) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimController = aimController;

        if (this.aimController != null) {
            this.aimController.setAlliance(GoalAimController.AllianceColor.RED);
        }
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void setIntakingActive(boolean intakingActive) {
        this.intakingActive = intakingActive;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    public void forceEnableAutoAlign() {
        autoAlignEnabled = true;
    }

    public void forceDisableAutoAlign() {
        autoAlignEnabled = false;
    }

    public void update(Gamepad gamepad1, Pose pose, long nowMs) {
        if (pose == null) return;

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        if (btnOptions.wasPressed(gamepad1.options)) {
            resetRobotPose();
            return;
        }

        if (btnTriangle.wasPressed(gamepad1.triangle)) {
            // 🔁 MIRRORED BLUE (Y flipped)
            relocalizeRobotPose(57.5, 90, Math.toRadians(3.8));
            return;
        }

        if (btnCross.wasPressed(gamepad1.cross)) {
            // 🔁 MIRRORED BLUE
            relocalizeRobotPose(-62, 16, 0);
            return;
        }

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

        if (autoAlignEnabled && aimController != null) {
            double autoTurn = aimController.getTurnPower();
            autoTurn = clamp(autoTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            lastVisionTurn = autoTurn;
            turn = autoTurn;
            usingAutoTurn = true;
            lastTurnSource = "ODOM_PD";
        } else {
            lastTurnSource = "MANUAL";
        }

        applyScaledDrive(drive, strafe, turn, usingAutoTurn);
    }

    private void resetRobotPose() {
        follower.startTeleopDrive();

        // 🔁 MIRRORED BLUE START POSE
        follower.setPose(new Pose(45.68, 130.75, Math.toRadians(-143.6)));

        if (aimController != null) {
            aimController.reset();
            aimController.setAlliance(GoalAimController.AllianceColor.RED);
            aimController.setRobotPose(45.68, 130.75, Math.toRadians(-143.6));
        }

        lastTurnSource = "POSE_RESET";
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void relocalizeRobotPose(double x, double y, double headingRad) {
        follower.startTeleopDrive();
        follower.setPose(new Pose(x, y, headingRad));

        if (aimController != null) {
            aimController.reset();
            aimController.setAlliance(GoalAimController.AllianceColor.RED);
            aimController.setRobotPose(x, y, headingRad);
        }

        lastTurnSource = "POSE_RELOCALIZE";
        lastVisionTurn = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingAutoTurn) {
        double translationScale = slowMode ? NORMAL_SPEED : 1.0;
        double rotationScale = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.5);

        lastTranslationScale = translationScale;
        lastRotationScale = rotationScale;

        double scaledDrive = drive * translationScale;
        double scaledStrafe = strafe * translationScale;
        double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(scaledDrive, scaledStrafe, scaledTurn, false);
    }

    public void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("DC Slow Mode", slowMode);
        telemetry.addData("DC Auto Align", autoAlignEnabled);
        telemetry.addData("DC Intaking", intakingActive);

        telemetry.addData("DC Last Turn Source", lastTurnSource);
        telemetry.addData("DC Last Auto Turn", "%.3f", lastVisionTurn);
        telemetry.addData("DC Last Applied Turn", "%.3f", lastAppliedTurn);
        telemetry.addData("DC Last Drive", "%.3f", lastDrive);
        telemetry.addData("DC Last Strafe", "%.3f", lastStrafe);

        telemetry.addData("DC Translation Scale", "%.2f", lastTranslationScale);
        telemetry.addData("DC Rotation Scale", "%.2f", lastRotationScale);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public boolean isTriangleActive() {
        return false;
    }
}