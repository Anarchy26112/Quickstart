package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import java.util.Locale;

@Autonomous(name = "CloseBlueAuto", group = "Auto")
public class CloseBlueAuto extends OpMode {

    // =========================
    // Pedro Pathing
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    // =========================
    // State tracking
    // =========================
    private int pathState;

    // Cleaner state constants
    private static final int START_ANGLE32 = 0;
    private static final int WAIT_ANGLE32 = 1;
    private static final int START_INTAKE_MID = 2;
    private static final int WAIT_INTAKE_MID = 3;
    private static final int START_SECOND_TRIPLE = 4;
    private static final int WAIT_SECOND_TRIPLE = 5;
    private static final int START_PUSH_GATE = 6;
    private static final int WAIT_PUSH_GATE = 7;
    private static final int DONE = -1;

    // =========================
    // Starting pose + path points
    // =========================
    private final Pose startPose = new Pose(124, 45, Math.toRadians(-140));

    public static Pose finalPose;

    private final Pose Intake2nd = new Pose(48, 20, Math.toRadians(90));
    private final Pose Shoot = new Pose(95.6, 14, Math.toRadians(-127.6));
    private final Pose Collected2nd = new Pose(48, 48, Math.toRadians(90));
    private final Pose pushGatePt = new Pose(67, 53, Math.toRadians(180));
    private final Pose PushCycle = new Pose(60, 54, Math.toRadians(30));

    // =========================
    // Paths
    // =========================
    private PathChain ShootFirst;
    private PathChain goToIntakeSecond;
    private PathChain intakeSecondTriple;
    private PathChain pushGate;
    private PathChain ShootSecond;
    private PathChain pushGateCycle;


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(START_ANGLE32);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Test", false);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            PoseHandoff.save(follower.getPose());
            finalPose = follower.getPose();
        }

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // =========================
    // PATH BUILDING
    // =========================
    public void buildPaths() {
        ShootFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        goToIntakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Intake2nd))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Intake2nd.getHeading())
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(Intake2nd, Collected2nd))
                .setLinearHeadingInterpolation(Intake2nd.getHeading(), Collected2nd.getHeading())
                .build();

        pushGate = follower.pathBuilder()
                .addPath(new BezierLine(Collected2nd, pushGatePt))
                .setLinearHeadingInterpolation(Collected2nd.getHeading(), pushGatePt.getHeading())
                .build();
        ShootSecond = follower.pathBuilder()
                .addPath(new BezierLine(pushGatePt, Shoot))
                .setLinearHeadingInterpolation(pushGatePt.getHeading(), Shoot.getHeading())
                .build();
        pushGateCycle = follower.pathBuilder()
                .addPath(new BezierLine(Intake2nd, PushCycle))
                .setLinearHeadingInterpolation(Intake2nd.getHeading(), PushCycle.getHeading())
                .build();
    }

    // =========================
    // CLEANER AUTO STATE MACHINE
    // =========================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(ShootFirst, true);
                setPathState(WAIT_ANGLE32);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(goToIntakeSecond, true);
                    setPathState(2);
                }
                break;

            case 2:
                follower.followPath(intakeSecondTriple, true);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pushGate, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(ShootSecond, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(goToIntakeSecond, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pushGateCycle, true);
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected Follower getFollower() {
        return follower;
    }
}