package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import java.util.Locale;

@Autonomous(name = "FarBlueAuto", group = "Auto")
public class FarBlueAuto extends OpMode {

    // =========================
    // Pedro Pathing
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    // =========================
    // Auto manipulator
    // =========================
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoManipulator autoManipulator;

    // =========================
    // State tracking
    // =========================
    private int pathState;

    // =========================
    // Starting pose + path points
    // =========================
    private final Pose startPose = new Pose(21, 0, Math.toRadians(-90));

    public static Pose finalPose;

    private final Pose IntakeA = new Pose(25, -75, Math.toRadians(0));
    private final Pose IntakeB = new Pose(25, -51, Math.toRadians(0));
    private final Pose IntakeC = new Pose(25, -27, Math.toRadians(0));
    private final Pose CollectedA = new Pose(52.5, -75, Math.toRadians(0));
    private final Pose CollectedB = new Pose(59, -51, Math.toRadians(0));
    private final Pose CollectedC = new Pose(59, -27, Math.toRadians(0));

    private final Pose Shoot = new Pose(19.4, -4, Math.toRadians(110));
    private final Pose IntakeHP = new Pose(57,-3,Math.toRadians(0));//change this use telem
    private final Pose CollectedHP = new Pose(61,-3,Math.toRadians(0));//change this use telem

    private final Pose pushGatePt = new Pose(59.5, -58, Math.toRadians(90));
    private final Pose shootBMidPt = new Pose(22, -58, Math.toRadians(90));
    private final Pose PushCycle = new Pose(53, -60, Math.toRadians(0));
    private final Pose CycleCollect = new Pose(61.5,-37, Math.toRadians(-67));
    private final Pose CycleCollected = new Pose(61.5, -47,Math.toRadians(-67));

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
    private PathChain GateCycle;
    private PathChain GateCycleCollect;
    private PathChain GateCycleCollected;
    private PathChain GateCycleShoot;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        autoManipulator = new AutoManipulator(intake, gate, telemetry);

        autoManipulator.setIntakePower(1.0, 1.0);
        autoManipulator.setHoldingPower(1.0);
        autoManipulator.setShootingFeedPower(1.0, 1.0);
        autoManipulator.setShootingTimings(400, 1200);

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
        autoManipulator.addTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);//0

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autoManipulator.update();
        autonomousPathUpdate();

        shooter.update();
        shooter.setVelocity(1620);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Test: ", false);
        autoManipulator.addTelemetry();

        telemetry.update();
    }

    @Override
    public void stop() {
        if (autoManipulator != null) {
            autoManipulator.stopAll();
        }

        if (follower != null) {
            PoseHandoff.save(follower.getPose());
            finalPose = follower.getPose();
        }

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

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
                .addPath(new BezierCurve(pushGatePt, shootBMidPt, Shoot))
                .setLinearHeadingInterpolation(pushGatePt.getHeading(), Shoot.getHeading())
                .build();

        ShootA = follower.pathBuilder()
                .addPath(new BezierLine(CollectedA, Shoot))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), Shoot.getHeading())
                .build();

        ShootC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, Shoot))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), Shoot.getHeading())
                .build();
        GateCycle = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeB))
                .setLinearHeadingInterpolation(Shoot.getHeading(), PushCycle.getHeading())
                .addPath(new BezierLine(IntakeB,PushCycle))
                .setLinearHeadingInterpolation(IntakeB.getHeading(), PushCycle.getHeading())//intakeB.getheading, pushcycle.getheading
                .build();
        GateCycleCollect = follower.pathBuilder()
                .addPath(new BezierLine(PushCycle, CycleCollect))
                .setLinearHeadingInterpolation(PushCycle.getHeading(), CycleCollect.getHeading())
                .build();
        GateCycleCollected = follower.pathBuilder()
                .addPath(new BezierLine(CycleCollect, CycleCollected))
                .setConstantHeadingInterpolation(CycleCollect.getHeading())
                .build();
        GateCycleShoot = follower.pathBuilder()
                .addPath(new BezierLine(CycleCollect, Shoot))
                .setLinearHeadingInterpolation(CycleCollect.getHeading(), Shoot.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                autoManipulator.hold();
                follower.followPath(ShootPreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(2);
                }
                break;

            case 2:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeSecond, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondTriple, 1.0, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(pushGate, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(ShootB, true);
                    setPathState(6);
                }
                break;

            case -1:
            default:
                autoManipulator.idle();
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