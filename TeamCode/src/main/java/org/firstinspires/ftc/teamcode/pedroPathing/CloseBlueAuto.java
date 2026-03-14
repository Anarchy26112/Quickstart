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
    private final Pose startPose = new Pose(45, -124, Math.toRadians(140));

    public static Pose finalPose;

    private final Pose IntakeA = new Pose(25,-75,Math.toRadians(0));
    private final Pose IntakeB = new Pose(25, -51, Math.toRadians(0));
    private final Pose IntakeC = new Pose(25, -27, Math.toRadians(0));
    private final Pose CollectedA = new Pose(51, -75, Math.toRadians(0));
    private final Pose CollectedB = new Pose(53, -51, Math.toRadians(0));
    private final Pose CollectedC = new Pose(53, -27, Math.toRadians(0));

    private final Pose Shoot = new Pose(20, -90, Math.toRadians(136.06));
    private final Pose pushGatePt = new Pose(56, -58, Math.toRadians(90));//make sure to pause here for the balls to come out
    private final Pose shootBMidPt = new Pose(22, -58, Math.toRadians(90));
    private final Pose PushCycle = new Pose(60, 54, Math.toRadians(-60));

    // =========================
    // Paths
    // =========================
    private PathChain ShootPreload;
    private PathChain goToIntakeSecond;
    private PathChain intakeSecondTriple;
    private PathChain goToIntakeFirst;
    private PathChain intakeFirstTriple;
    private PathChain goToIntakeThird;
    private PathChain intakeThirdTriple;
    private PathChain pushGate;
    private PathChain ShootB;
    private PathChain ShootA;
    private PathChain ShootC;
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
        setPathState(START_ANGLE32);//START_ANGLE32

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
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        goToIntakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeB))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeB.getHeading())
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeB, CollectedB))
                .setLinearHeadingInterpolation(IntakeB.getHeading(), CollectedB.getHeading())
                .build();

        goToIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeA))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeA.getHeading())
                .build();

        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeA, CollectedA))
                .setLinearHeadingInterpolation(IntakeA.getHeading(), CollectedA.getHeading())
                .build();

        goToIntakeThird = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeC))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeC.getHeading())
                .build();

        intakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeC, CollectedC))
                .setLinearHeadingInterpolation(IntakeC.getHeading(), CollectedC.getHeading())
                .build();

        pushGate = follower.pathBuilder()
                .addPath(new BezierLine(CollectedB, pushGatePt))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), pushGatePt.getHeading())
                .build();
        ShootB = follower.pathBuilder()
                .addPath(new BezierLine(pushGatePt, shootBMidPt))
                .setLinearHeadingInterpolation(pushGatePt.getHeading(), shootBMidPt.getHeading())
                .addPath(new BezierLine(shootBMidPt,Shoot))
                .setLinearHeadingInterpolation(shootBMidPt.getHeading(), Shoot.getHeading())

                .build();

        ShootA = follower.pathBuilder()
                .addPath(new BezierLine(CollectedA, Shoot))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), Shoot.getHeading())
                .build();

        ShootC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, Shoot))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), Shoot.getHeading())
                .build();

        pushGateCycle = follower.pathBuilder()
                .addPath(new BezierLine(IntakeB, PushCycle))
                .setLinearHeadingInterpolation(IntakeB.getHeading(), PushCycle.getHeading())
                .build();
    }

    // =========================
    // CLEANER AUTO STATE MACHINE
    // =========================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(ShootPreload, true);
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
                    follower.followPath(ShootB, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(goToIntakeFirst, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstTriple, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                follower.followPath(ShootA, true);
                setPathState(8);
            }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(goToIntakeThird, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(intakeThirdTriple, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(ShootC, true);
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