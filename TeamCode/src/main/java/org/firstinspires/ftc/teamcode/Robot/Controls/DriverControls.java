package org.firstinspires.ftc.teamcode.Robot.Controls;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.ButtonHelper;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;
import org.firstinspires.ftc.teamcode.Robot.GoalAimController;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class DriverControls {

    private final Follower          follower;
    private final Telemetry         telemetry;
    private final GoalAimController aimController;
    private final Alliance          alliance;
    private final double            allianceMultiplier;

    private boolean slowMode         = false;
    private boolean autoAlignEnabled = false;
    private boolean intakingActive   = false;

    private final ButtonHelper btnTouchpad = new ButtonHelper();
    private final ButtonHelper btnPS       = new ButtonHelper();
    private final ButtonHelper btnOptions  = new ButtonHelper();
    private final ButtonHelper btnTriangle = new ButtonHelper();
    private final ButtonHelper btnCross    = new ButtonHelper();

    private double cachedDrive      = 0.0;
    private double cachedStrafe     = 0.0;
    private double cachedManualTurn = 0.0;

    private double lastVisionTurn       = 0.0;
    private double lastAppliedTurn      = 0.0;
    private double lastTranslationScale = 1.0;
    private double lastRotationScale    = 1.0;

    private byte lastTurnSource = 0;  // 0=MANUAL, 1=ODOM_PD, 2=POSE_RESET, 3=POSE_RELOCALIZE
    private static final byte TURN_SOURCE_MANUAL = 0;
    private static final byte TURN_SOURCE_ODOM_PD = 1;
    private static final byte TURN_SOURCE_POSE_RESET = 2;
    private static final byte TURN_SOURCE_POSE_RELOCALIZE = 3;
    private static final String[] TURN_SOURCE_NAMES = {"MANUAL", "ODOM_PD", "POSE_RESET", "POSE_RELOCALIZE"};

    private boolean resetPoseRequested    = false;
    private boolean trianglePoseRequested = false;
    private boolean crossPoseRequested    = false;

    public DriverControls(Follower follower, Telemetry telemetry, GoalAimController aimController, Alliance alliance) {
        this.follower           = follower;
        this.telemetry          = telemetry;
        this.aimController      = aimController;
        this.alliance           = alliance;
        this.allianceMultiplier = alliance == Alliance.BLUE ? -1.0 : 1.0;
        setAimAlliance();
    }

    public void startTeleopDrive()            { follower.startTeleopDrive(); }
    public void setIntakingActive(boolean a)  { intakingActive = a; }
    public boolean isAutoAlignEnabled()       { return autoAlignEnabled; }
    public void forceEnableAutoAlign()        { autoAlignEnabled = true; }
    public void forceDisableAutoAlign()       { autoAlignEnabled = false; }

    public void readInputs(Gamepad gamepad1) {
        resetPoseRequested    = false;
        trianglePoseRequested = false;
        crossPoseRequested    = false;

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) autoAlignEnabled = !autoAlignEnabled;
        if (btnPS.wasPressed(gamepad1.ps))             slowMode         = !slowMode;

        if (btnOptions.wasPressed(gamepad1.options))    resetPoseRequested    = true;
        else if (btnTriangle.wasPressed(gamepad1.triangle)) trianglePoseRequested = true;
        else if (btnCross.wasPressed(gamepad1.cross))       crossPoseRequested    = true;

        final double mult = allianceMultiplier;
        cachedDrive      = -gamepad1.left_stick_y * mult;
        cachedStrafe     = -gamepad1.left_stick_x * mult;
        cachedManualTurn = -gamepad1.right_stick_x;
    }

    public boolean handlePoseRequests() {
        if (resetPoseRequested) {
            resetRobotPose();
            resetPoseRequested = false;
            return true;
        }
        if (trianglePoseRequested) {
            relocalizeRobotPose(FieldMirror.pose(alliance, 38.0, 134.0, 90.0));
            trianglePoseRequested = false;
            return true;
        }
        if (crossPoseRequested) {
            relocalizeRobotPose(FieldMirror.pose(alliance, 134.49498648652067, 16.28776978417272, 180.0));
            crossPoseRequested = false;
            return true;
        }
        return false;
    }

    public void applyDrive(boolean usingAutoTurn) {
        double turn;
        if (usingAutoTurn) {
            double autoTurn = aimController.getTurnPower();
            if      (autoTurn >  MAX_AUTO_TURN) autoTurn =  MAX_AUTO_TURN;
            else if (autoTurn < -MAX_AUTO_TURN) autoTurn = -MAX_AUTO_TURN;
            lastVisionTurn = autoTurn;
            turn           = autoTurn;
            lastTurnSource = TURN_SOURCE_ODOM_PD;
        } else {
            lastVisionTurn = 0.0;
            turn           = cachedManualTurn;
            lastTurnSource = TURN_SOURCE_MANUAL;
        }
        applyScaledDrive(cachedDrive, cachedStrafe, turn, usingAutoTurn);
    }

    private void resetRobotPose() {
        follower.startTeleopDrive();
        final Pose resetPose = FieldMirror.pose(alliance, 26.32, 130.75, 143.6);
        follower.setPose(resetPose);
        aimController.reset();
        setAimAlliance();
        aimController.setRobotPose(resetPose.getX(), resetPose.getY(), resetPose.getHeading());
        lastTurnSource = TURN_SOURCE_POSE_RESET;
        lastVisionTurn = lastAppliedTurn = 0.0;
    }

    private void relocalizeRobotPose(Pose pose) {
        follower.startTeleopDrive();
        follower.setPose(pose);
        aimController.reset();
        setAimAlliance();
        aimController.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());
        lastTurnSource = TURN_SOURCE_POSE_RELOCALIZE;
        lastVisionTurn = lastAppliedTurn = 0.0;
    }

    private void applyScaledDrive(double drive, double strafe, double turn, boolean usingAutoTurn) {
        final double translationScale = slowMode ? NORMAL_SPEED : 1.0;
        final double rotationScale    = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.50);

        lastTranslationScale = translationScale;
        lastRotationScale    = rotationScale;

        final double scaledTurn = turn * rotationScale;
        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(drive * translationScale, strafe * translationScale, scaledTurn, false);
    }

    private void setAimAlliance() {
        aimController.setAlliance(
                alliance == Alliance.BLUE
                        ? GoalAimController.AllianceColor.BLUE
                        : GoalAimController.AllianceColor.RED);
    }

    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Driver Slow Mode",         slowMode);
        telemetry.addData("Driver Auto Align",        autoAlignEnabled);
        telemetry.addData("Driver Intaking",          intakingActive);
        telemetry.addData("Driver Turn Source",       TURN_SOURCE_NAMES[lastTurnSource]);
        telemetry.addData("Driver Drive",             cachedDrive);
        telemetry.addData("Driver Strafe",            cachedStrafe);
        telemetry.addData("Driver Vision Turn",       lastVisionTurn);
        telemetry.addData("Driver Applied Turn",      lastAppliedTurn);
        telemetry.addData("Driver Translation Scale", lastTranslationScale);
        telemetry.addData("Driver Rotation Scale",    lastRotationScale);
    }
}