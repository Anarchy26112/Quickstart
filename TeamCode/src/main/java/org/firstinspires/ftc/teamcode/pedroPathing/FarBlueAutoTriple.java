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

@Autonomous(name = "FarBlueAutoTriple", group = "Auto")
public class FarBlueAutoTriple extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoManipulator autoManipulator;

    private int pathState;

    private final Pose startPose = new Pose(21, 0, Math.toRadians(-90));

    public static Pose finalPose;

    private final Pose IntakeC = new Pose(25, -27, Math.toRadians(0));
    private final Pose CollectedC = new Pose(62, -27, Math.toRadians(0));

    private final Pose Shoot = new Pose(19.4, -8, Math.toRadians(121));
    private final Pose Shoot2 = new Pose(19.4, -8, Math.toRadians(112.333));
    private final Pose Shoot3 = new Pose(19.4, -8, Math.toRadians(112.33));


    private final Pose IntakeHP = new Pose(57, -6, Math.toRadians(22.5));
    private final Pose CollectedHP = new Pose(64, -1, Math.toRadians(22.5));
    private final Pose IntakeHP2 = new Pose(57, -13, Math.toRadians(45));
    private final Pose CollectedHP2 = new Pose(62, -3, Math.toRadians(45));
    private final Pose HPCornerMid = new Pose(36, -20, Math.toRadians(122));
    private final Pose Out = new Pose(37, 0, Math.toRadians(0));

    private PathChain ShootPreload;
    private PathChain ShootC;

    private PathChain goToIntakeThird;
    private PathChain intakeThirdTriple;
    private PathChain HP1;
    private PathChain HP2;

    private PathChain ShootHP;
    private PathChain ShootHP2;

    private PathChain GoIn1;
    private PathChain GoOut1;

    private PathChain GoIn2;
    private PathChain GoOut2;
    private PathChain Tran;
    private PathChain Leave;

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
        setPathState(0);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autoManipulator.update();
        autonomousPathUpdate();

        shooter.update();
        shooter.setVelocity(1930);

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

        goToIntakeThird = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeC))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeC.getHeading())
                .build();

        intakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeC, CollectedC))
                .setLinearHeadingInterpolation(IntakeC.getHeading(), CollectedC.getHeading())
                .build();

        ShootC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, Shoot3))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), Shoot3.getHeading())
                .build();

        HP1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeHP))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeHP.getHeading())
                .build();

        GoIn1 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP, CollectedHP))
                .setConstantHeadingInterpolation(IntakeHP.getHeading())
                .build();

        GoOut1 = follower.pathBuilder()
                .addPath(new BezierLine(CollectedHP, IntakeHP))
                .setConstantHeadingInterpolation(IntakeHP.getHeading())
                .build();

        HP2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeHP2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeHP2.getHeading())
                .build();

        GoIn2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, CollectedHP2))
                .setConstantHeadingInterpolation(IntakeHP2.getHeading())
                .build();

        GoOut2 = follower.pathBuilder()
                .addPath(new BezierLine(CollectedHP2, IntakeHP2))
                .setConstantHeadingInterpolation(IntakeHP2.getHeading())
                .build();

        Tran = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, IntakeHP))
                .setLinearHeadingInterpolation(IntakeHP2.getHeading(), IntakeHP.getHeading())
                .build();

        ShootHP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakeHP, HPCornerMid, Shoot2))
                .setLinearHeadingInterpolation(IntakeHP.getHeading(), Shoot2.getHeading())
                .build();
        ShootHP2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, Shoot2))
                .setLinearHeadingInterpolation(IntakeHP.getHeading(), Shoot2.getHeading())
                .build();

        Leave = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, Out))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), Out.getHeading())
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
                    follower.followPath(HP1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(GoIn1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    follower.followPath(GoOut1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(ShootHP, 0.7, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(9);
                }
                break;

            case 9:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeThird, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(intakeThirdTriple, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    follower.followPath(ShootC, 0.73,true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(HP2, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(GoIn2, true);
                    setPathState(15);
                }
                break;

            case 15:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.4) || pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(GoOut2, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(ShootHP2, 0.73, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(21);
                }
                break;
            case 21:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(HP2, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(GoIn2, true);
                    setPathState(23);
                }
                break;

            case 23:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.4) || pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(GoOut2, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(ShootHP2, 0.73, true);
                    setPathState(25);
                }
                break;

            case 25:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(26);
                }
                break;

            case 26:
                if (autoManipulator.isShootComplete()) {
                    follower.followPath(Leave, true);
                    setPathState(27);
                }
                break;

            case 27:
                if (!follower.isBusy()) {
                    setPathState(-1);
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