package org.firstinspires.ftc.teamcode.Robot.Controls;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    private final ButtonHelper btnCircle = new ButtonHelper();
    private final ButtonHelper btnOptions = new ButtonHelper();

    // 🔁 MIRRORED
    private final Pose parkingRed = new Pose(-25, 30, 0);
    private PathChain parkingRedPath;

    private boolean homingMechanismEngaged = false;
    private boolean homingPathStarted = false;

    private double lastVisionTurn = 0.0;
    private double lastAppliedTurn = 0.0;
    private double lastDrive = 0.0;
    private double lastStrafe = 0.0;

    private double lastTranslationScale = 1.0;
    private double lastRotationScale = 1.0;
    private String lastTurnSource = "MANUAL";

    // 🔁 MIRRORED
    private static final double INTAKE_AIM_TARGET_DEG = 28.0;
    private static final double INTAKE_AIM_TARGET_RAD = Math.toRadians(INTAKE_AIM_TARGET_DEG);
    private static final double INTAKE_AIM_MAX_TURN = 0.35;
    private static final double INTAKE_AIM_DEADBAND_RAD = Math.toRadians(1.0);

    public DriverControlsRed(Follower follower, Telemetry telemetry, GoalAimController aimController) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimController = aimController;

        if (this.aimController != null) {
            this.aimController.setAlliance(GoalAimController.AllianceColor.RED);
        }

        buildParkingPath();
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

        if (btnCircle.wasPressed(gamepad1.circle)) {
            homingMechanismEngaged = !homingMechanismEngaged;

            if (homingMechanismEngaged) {
                follower.followPath(parkingRedPath);
                homingPathStarted = true;
            } else {
                homingPathStarted = false;
                startTeleopDrive();
            }
        }

        if (homingMechanismEngaged) {
            lastTurnSource = "HOMING_PATH";
            return;
        }

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        lastDrive = drive;
        lastStrafe = strafe;

        boolean usingAutoTurn = false;

        boolean squareAimActive = gamepad1.square && intakingActive;

        if (squareAimActive) {
            double currentHeading = pose.getHeading();
            double headingError = wrapAngleRad(INTAKE_AIM_TARGET_RAD - currentHeading);

            double turnCommand = HEADING_kP * headingError;

            if (Math.abs(headingError) > INTAKE_AIM_DEADBAND_RAD) {
                turnCommand += Math.signum(headingError) * FAST_kS_VOLTAGE_COMP;
            }

            turn = clamp(turnCommand, -INTAKE_AIM_MAX_TURN, INTAKE_AIM_MAX_TURN);
            usingAutoTurn = true;
            lastVisionTurn = turn;
            lastTurnSource = "INTAKE_30deg";

            applyScaledDrive(drive, strafe, turn, usingAutoTurn);
            return;
        }

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
        homingMechanismEngaged = false;
        homingPathStarted = false;

        follower.startTeleopDrive();

        follower.setPose(new Pose(45, 120, Math.toRadians(-140)));

        if (aimController != null) {
            aimController.reset();
            aimController.setAlliance(GoalAimController.AllianceColor.RED);
            aimController.setRobotPose(45, 120, Math.toRadians(-140));
        }

        lastTurnSource = "POSE_RESET";
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

    private void buildParkingPath() {
        Pose startPose = new Pose(45, 120, Math.toRadians(-140));
        parkingRedPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkingRed))
                .build();
    }

    private static double wrapAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public boolean isTriangleActive() {
        return false;
    }
}