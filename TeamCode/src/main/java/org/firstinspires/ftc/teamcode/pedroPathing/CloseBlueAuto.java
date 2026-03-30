package org.firstinspires.ftc.teamcode.pedroPathing;

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

@Autonomous(name = "CloseBlueAuto", group = "Auto")
public class CloseBlueAuto extends OpMode {

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
    // Tunables
    // =========================
    private static final double SHOOT_SETTLE_TIME = 0.15;
    private static final double GATE_COLLECT_SETTLE_TIME = 0.5;
    private static final double GATE_CYCLE_TIME = 0.2;
    private static final double SHOOTER_VELOCITY = 1570;

    // Shared and adjusted shot headings
    private static final double SHOOT_MAIN_HEADING = Math.toRadians(127);
    private static final double SHOOT_LEFT_HEADING = Math.toRadians(135);
    private static final double SHOOT_LEFT_HEADING_MORE = Math.toRadians(140); // first + third shots


    // =========================
    // Starting pose + path points
    // =========================
    private final Pose startPose = new Pose(45, -124, Math.toRadians(140));

    public static Pose finalPose;

    private final Pose IntakeA = new Pose(25, -75, Math.toRadians(0));
    private final Pose IntakeB = new Pose(25, -51, Math.toRadians(0));
    private final Pose IntakeC = new Pose(25, -27, Math.toRadians(0));

    private final Pose CollectedA = new Pose(54.5, -75, Math.toRadians(0));
    private final Pose CollectedB = new Pose(62, -51, Math.toRadians(0));
    private final Pose CollectedC = new Pose(62, -27, Math.toRadians(0));

    // Only two shooting poses
    private final Pose SHOOT_MAIN = new Pose(20, -84, SHOOT_MAIN_HEADING);
    private final Pose SHOOT_FINAL = new Pose(17, -100, Math.toRadians(136));

    private final Pose shootBMidPt = new Pose(18, -58, Math.toRadians(90));
    private final Pose PushCycle = new Pose(53.6, -63.5, Math.toRadians(0));
    private final Pose gateCycleMid = new Pose(24.2, -61, Math.toRadians(70));
    private final Pose CycleCollect = new Pose(64, -43.5, Math.toRadians(-67));
    private final Pose CycleCollected = new Pose(64, -47, Math.toRadians(-67));

    // =========================
    // Paths
    // =========================
    private PathChain shootPreload;

    private PathChain goToIntakeSecond;
    private PathChain intakeSecondTriple;
    private PathChain shootFromB;

    private PathChain gateCycle;
    private PathChain gateCycleCollect;
    private PathChain gateCycleCollected;
    private PathChain gateCycleShoot;

    private PathChain goToIntakeThird;
    private PathChain intakeThirdTriple;
    private PathChain shootFromC;

    private PathChain goToIntakeFirst;
    private PathChain intakeFirstTriple;
    private PathChain shootFromAFinal;

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

        shooter.setVelocity(SHOOTER_VELOCITY);
        shooter.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
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
        // First shot: slightly more left
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SHOOT_MAIN))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT_LEFT_HEADING)
                .build();

        goToIntakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_MAIN, IntakeB))
                .setLinearHeadingInterpolation(SHOOT_MAIN.getHeading(), IntakeB.getHeading())
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeB, CollectedB))
                .setLinearHeadingInterpolation(IntakeB.getHeading(), CollectedB.getHeading())
                .build();

        shootFromB = follower.pathBuilder()
                .addPath(new BezierCurve(CollectedB, shootBMidPt, SHOOT_MAIN))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), SHOOT_MAIN.getHeading())
                .build();

        gateCycle = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_MAIN, gateCycleMid, PushCycle))
                .setLinearHeadingInterpolation(SHOOT_MAIN.getHeading(), PushCycle.getHeading())
                .build();

        gateCycleCollect = follower.pathBuilder()
                .addPath(new BezierLine(PushCycle, CycleCollect))
                .setLinearHeadingInterpolation(PushCycle.getHeading(), CycleCollect.getHeading())
                .build();

        gateCycleCollected = follower.pathBuilder()
                .addPath(new BezierLine(CycleCollect, CycleCollected))
                .setConstantHeadingInterpolation(CycleCollect.getHeading())
                .build();

        gateCycleShoot = follower.pathBuilder()
                .addPath(new BezierCurve(CycleCollected, shootBMidPt, SHOOT_MAIN))
                .setLinearHeadingInterpolation(CycleCollected.getHeading(), SHOOT_LEFT_HEADING_MORE)
                .build();

        goToIntakeThird = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_MAIN, IntakeC))
                .setLinearHeadingInterpolation(SHOOT_MAIN.getHeading(), IntakeC.getHeading())
                .build();

        intakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeC, CollectedC))
                .setLinearHeadingInterpolation(IntakeC.getHeading(), CollectedC.getHeading())
                .build();

        shootFromC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, SHOOT_MAIN))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), SHOOT_MAIN.getHeading())
                .build();

        goToIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_MAIN, IntakeA))
                .setLinearHeadingInterpolation(SHOOT_MAIN.getHeading(), IntakeA.getHeading())
                .build();

        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(IntakeA, CollectedA))
                .setLinearHeadingInterpolation(IntakeA.getHeading(), CollectedA.getHeading())
                .build();

        shootFromAFinal = follower.pathBuilder()
                .addPath(new BezierLine(CollectedA, SHOOT_FINAL))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), SHOOT_FINAL.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // =========================
            // Preload shot
            // =========================
            case 0:
                autoManipulator.hold();
                follower.followPath(shootPreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(3);
                }
                break;

            case 3:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeSecond, true);
                    setPathState(4);
                }
                break;

            // =========================
            // Collect second, shoot main
            // =========================
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondTriple, 1.0, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromB, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycle, true);
                    setPathState(9);
                }
                break;

            // =========================
            // Gate cycle, hold, collect, return to main shoot
            // =========================
            case 9:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleCollect, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(gateCycleCollected, 1.9, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_COLLECT_SETTLE_TIME) {
                    autoManipulator.hold();
                    follower.followPath(gateCycleShoot, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(16);
                }
                break;

            case 16:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeThird, true);
                    setPathState(-2);
                }
                break;
            case -2:
                if (!follower.isBusy()) {
                    follower.followPath(intakeThirdTriple, 1.0, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromC, true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(20);
                }
                break;

            case 20:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeFirst, true);
                    setPathState(21);
                }
                break;

            // =========================
            // Collect first, shoot final
            // =========================
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstTriple, 1.0, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromAFinal, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    setPathState(24);
                }
                break;

            case 24:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(25);
                }
                break;

            case 25:
                if (autoManipulator.isShootComplete()) {
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