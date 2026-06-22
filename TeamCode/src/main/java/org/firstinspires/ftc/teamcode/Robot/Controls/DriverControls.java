package org.firstinspires.ftc.teamcode.Robot.Controls;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

    private byte lastTurnSource = 0;

    private static final byte TURN_SOURCE_MANUAL          = 0;
    private static final byte TURN_SOURCE_ODOM_PD         = 1;
    private static final byte TURN_SOURCE_POSE_RESET      = 2;
    private static final byte TURN_SOURCE_POSE_RELOCALIZE = 3;
    private static final byte TURN_SOURCE_DPAD_PATH       = 4;

    private static final String[] TURN_SOURCE_NAMES = {
            "MANUAL",
            "ODOM_PD",
            "POSE_RESET",
            "POSE_RELOCALIZE",
            "DPAD_PATH"
    };

    private boolean resetPoseRequested    = false;
    private boolean trianglePoseRequested = false;
    private boolean crossPoseRequested    = false;

    // ========== DPAD PEDRO CURVE TARGET ==========
    //
    // Blue/reference-side points.
    // Red will be mirrored by FieldMirror.pose(...).
    private static final double DPAD_CONTROL_X_BLUE = 41.41;
    private static final double DPAD_CONTROL_Y_BLUE = 57.8;
    private static final double DPAD_CONTROL_HEADING_DEG_BLUE = 150.0;

    private static final double DPAD_TARGET_X_BLUE = 12.5;
    private static final double DPAD_TARGET_Y_BLUE = 57.8;
    private static final double DPAD_TARGET_HEADING_DEG_BLUE = 150.0;

    private boolean dpadPathHeld   = false;
    private boolean dpadPathActive = false;

    public DriverControls(
            Follower follower,
            Telemetry telemetry,
            GoalAimController aimController,
            Alliance alliance
    ) {
        this.follower           = follower;
        this.telemetry          = telemetry;
        this.aimController      = aimController;
        this.alliance           = alliance;
        this.allianceMultiplier = alliance == Alliance.BLUE ? -1.0 : 1.0;
        setAimAlliance();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void setIntakingActive(boolean a) {
        intakingActive = a;
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

    public boolean isPathOverrideActive() {
        return dpadPathHeld || dpadPathActive;
    }

    public boolean isDpadPathActive() {
        return dpadPathActive;
    }

    public void readInputs(Gamepad gamepad1) {
        resetPoseRequested    = false;
        trianglePoseRequested = false;
        crossPoseRequested    = false;

        if (btnTouchpad.wasPressed(gamepad1.touchpad)) {
            autoAlignEnabled = !autoAlignEnabled;
        }

        if (btnPS.wasPressed(gamepad1.ps)) {
            slowMode = !slowMode;
        }

        if (btnOptions.wasPressed(gamepad1.options)) {
            resetPoseRequested = true;
        } else if (btnTriangle.wasPressed(gamepad1.triangle)) {
            trianglePoseRequested = true;
        } else if (btnCross.wasPressed(gamepad1.cross)) {
            crossPoseRequested = true;
        }

        dpadPathHeld =
                gamepad1.dpad_up
                        || gamepad1.dpad_down
                        || gamepad1.dpad_left
                        || gamepad1.dpad_right;

        // Dpad pathing has full priority.
        // If any dpad is held, auto-align is forced OFF and stays OFF
        // until the driver manually turns it back on with touchpad.
        if (dpadPathHeld) {
            autoAlignEnabled = false;
        }

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
            relocalizeRobotPose(FieldMirror.pose(alliance, 14.5, 90.0, 178.63));
            trianglePoseRequested = false;
            return true;
        }

        if (crossPoseRequested) {
            relocalizeRobotPose(FieldMirror.pose(
                    alliance,
                    134.49498648652067,
                    16.28776978417272,
                    180.0
            ));
            crossPoseRequested = false;
            return true;
        }

        return false;
    }

    public void applyDrive(boolean usingAutoTurn, Pose currentPose) {
        if (dpadPathHeld) {
            applyDpadPathOverride(currentPose);
            return;
        }

        if (dpadPathActive) {
            cancelDpadPathAndReturnToTeleop();
        }

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

    private void applyDpadPathOverride(Pose currentPose) {
        if (currentPose == null) {
            return;
        }

        // Extra safety: auto-align cannot remain active during dpad pathing.
        autoAlignEnabled = false;

        if (!dpadPathActive) {
            startDpadCurveToTarget(currentPose);
        }

        lastTurnSource       = TURN_SOURCE_DPAD_PATH;
        lastVisionTurn       = 0.0;
        lastAppliedTurn      = 0.0;
        lastTranslationScale = 1.0;
        lastRotationScale    = 1.0;
    }

    private void startDpadCurveToTarget(Pose currentPose) {
        final Pose controlPose = FieldMirror.pose(
                alliance,
                DPAD_CONTROL_X_BLUE,
                DPAD_CONTROL_Y_BLUE,
                DPAD_CONTROL_HEADING_DEG_BLUE
        );

        final Pose targetPose = FieldMirror.pose(
                alliance,
                DPAD_TARGET_X_BLUE,
                DPAD_TARGET_Y_BLUE,
                DPAD_TARGET_HEADING_DEG_BLUE
        );

        final PathChain dpadPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        currentPose,
                        controlPose,
                        targetPose
                ))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();

        follower.followPath(dpadPath, true);

        dpadPathActive = true;
    }

    private void cancelDpadPathAndReturnToTeleop() {
        follower.breakFollowing();
        follower.startTeleopDrive();

        dpadPathActive = false;

        lastVisionTurn       = 0.0;
        lastAppliedTurn      = 0.0;
        lastTranslationScale = 1.0;
        lastRotationScale    = 1.0;
        lastTurnSource       = TURN_SOURCE_MANUAL;
    }

    public void cancelDriveOverrides() {
        if (dpadPathActive) {
            cancelDpadPathAndReturnToTeleop();
        }
    }

    private void resetRobotPose() {
        follower.startTeleopDrive();

        final Pose resetPose = FieldMirror.pose(alliance, 26.32, 130.75, 143.6);

        follower.setPose(resetPose);
        aimController.reset();
        setAimAlliance();
        aimController.setRobotPose(resetPose.getX(), resetPose.getY(), resetPose.getHeading());

        dpadPathActive = false;

        lastTurnSource  = TURN_SOURCE_POSE_RESET;
        lastVisionTurn  = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void relocalizeRobotPose(Pose pose) {
        follower.startTeleopDrive();

        follower.setPose(pose);
        aimController.reset();
        setAimAlliance();
        aimController.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());

        dpadPathActive = false;

        lastTurnSource  = TURN_SOURCE_POSE_RELOCALIZE;
        lastVisionTurn  = 0.0;
        lastAppliedTurn = 0.0;
    }

    private void applyScaledDrive(
            double drive,
            double strafe,
            double turn,
            boolean usingAutoTurn
    ) {
        final double translationScale = slowMode ? NORMAL_SPEED : 1.0;
        final double rotationScale    = usingAutoTurn ? 1.0 : (slowMode ? 0.20 : 0.50);

        lastTranslationScale = translationScale;
        lastRotationScale    = rotationScale;

        final double scaledTurn = turn * rotationScale;

        lastAppliedTurn = scaledTurn;

        follower.setTeleOpDrive(
                drive * translationScale,
                strafe * translationScale,
                scaledTurn,
                false
        );
    }

    private void setAimAlliance() {
        aimController.setAlliance(
                alliance == Alliance.BLUE
                        ? GoalAimController.AllianceColor.BLUE
                        : GoalAimController.AllianceColor.RED
        );
    }

    public void telemetry() {
        if (telemetry == null) return;

        telemetry.addData("Driver Slow Mode",         slowMode);
        telemetry.addData("Driver Auto Align",        autoAlignEnabled);
        telemetry.addData("Driver Intaking",          intakingActive);

        telemetry.addData("Driver Dpad Path Held",    dpadPathHeld);
        telemetry.addData("Driver Dpad Path Active",  dpadPathActive);
        telemetry.addData("Driver Path Override",     isPathOverrideActive());

        telemetry.addData("Driver Dpad Control X",    DPAD_CONTROL_X_BLUE);
        telemetry.addData("Driver Dpad Control Y",    DPAD_CONTROL_Y_BLUE);
        telemetry.addData("Driver Dpad Target X",     DPAD_TARGET_X_BLUE);
        telemetry.addData("Driver Dpad Target Y",     DPAD_TARGET_Y_BLUE);
        telemetry.addData("Driver Dpad Target Head",  DPAD_TARGET_HEADING_DEG_BLUE);

        telemetry.addData("Driver Turn Source",       TURN_SOURCE_NAMES[lastTurnSource]);
        telemetry.addData("Driver Drive",             cachedDrive);
        telemetry.addData("Driver Strafe",            cachedStrafe);
        telemetry.addData("Driver Vision Turn",       lastVisionTurn);
        telemetry.addData("Driver Applied Turn",      lastAppliedTurn);
        telemetry.addData("Driver Translation Scale", lastTranslationScale);
        telemetry.addData("Driver Rotation Scale",    lastRotationScale);
    }
}