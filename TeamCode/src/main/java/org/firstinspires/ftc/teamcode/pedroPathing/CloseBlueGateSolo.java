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

@Autonomous(name = "CloseBlueGateSolo", group = "Auto")
public class CloseBlueGateSolo extends OpMode {

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
    private static final double SHOOT_SETTLE_TIME = 0;
    private static final double GATE_COLLECT_SETTLE_TIME = 0;
    private static final double GATE_CYCLE_TIME = 1.17;
    private static final double SHOOTER_VELOCITY = 1540;

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
    private final Pose IntakeACurveMid = new Pose(8, -75, Math.toRadians(0));

    private final Pose IntakeB = new Pose(24, -51, Math.toRadians(0));
    private final Pose IntakeBCurveMid = new Pose(7, -51, Math.toRadians(0));

    private final Pose IntakeC = new Pose(25, -27, Math.toRadians(0));
    private final Pose IntakeCCurveMid = new Pose(8, -27, Math.toRadians(0));


    private final Pose CollectedA = new Pose(54.5, -75, Math.toRadians(0));
    private final Pose CollectedB = new Pose(62, -51, Math.toRadians(0));
    private final Pose CollectedC = new Pose(62, -27, Math.toRadians(0));

    // Only two shooting poses
    private final Pose SHOOT_MAIN = new Pose(20, -84, SHOOT_MAIN_HEADING);
    private final Pose SHOOT_FINAL = new Pose(14, -95, Math.toRadians(137.5));

    private final Pose shootBMidPt = new Pose(18, -58, Math.toRadians(90));
    private final Pose PushCycle = new Pose(55.55, -59.5, Math.toRadians(0));
    private final Pose ActualGateCyclePt = new Pose(59, -54.5, Math.toRadians(-34.35));//tune this point at the lab with real barriers

    private final Pose gateCycleMid = new Pose(24.2, -61, Math.toRadians(70));
    private final Pose CycleCollect = new Pose(60.5, -43.5, Math.toRadians(-67));
    private final Pose CycleCollected = new Pose(60.5, -47, Math.toRadians(-67));

    // =========================
    // Paths
    // =========================
    private PathChain shootPreload;

    private PathChain goToIntakeSecond;
    private PathChain intakeSecondTriple;
    private PathChain shootFromB;

    private PathChain gateCycle;
    private PathChain gateCycleActually;

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
        telemetry.addData("Test: ", true);

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
        telemetry.addData("Test: ", true);
        autoManipulator.addTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);//0 if you want to run the actual auto, -1 if you want telemetry

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


        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_MAIN, IntakeB, IntakeBCurveMid, CollectedB))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shootFromB = follower.pathBuilder()
                .addPath(new BezierCurve(CollectedB, shootBMidPt, SHOOT_MAIN))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), SHOOT_MAIN.getHeading())
                .build();

        gateCycleActually = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_MAIN, gateCycleMid, ActualGateCyclePt))
                .setLinearHeadingInterpolation(SHOOT_MAIN.getHeading(), ActualGateCyclePt.getHeading())
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


        intakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_MAIN, IntakeC, IntakeCCurveMid, CollectedC))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shootFromC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, SHOOT_MAIN))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), SHOOT_MAIN.getHeading())
                .build();


        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_MAIN, IntakeA, IntakeACurveMid, CollectedA))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
                    setPathState(4);//3
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
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
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
                    follower.followPath(gateCycleActually, true);
                    setPathState(9);
                }
                break;

            // =========================
            // Gate cycle, hold, collect, return to main shoot
            // =========================
            case 9:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                    //follower.followPath(gateCycleCollect, true);
                    setPathState(12);
                }
                break;

           /* case 11:
                if (!follower.isBusy()) {
                    follower.followPath(gateCycleCollected, 1.9, true);
                    setPathState(12);
                }
                break;*/

            case 12:
                if (!follower.isBusy()) {
                    autoManipulator.intake();

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

            /*case 16:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(goToIntakeThird, true);
                    setPathState(-2);
                }
                break;*/
            case 16:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(17);
                }
                break;

            // =========================
            // Gate cycle, hold, collect, return to main shoot
            // =========================
            case 17:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                   // follower.followPath(gateCycleCollect, true);
                    setPathState(20);
                }
                break;

        /*    case 19:
                if (!follower.isBusy()) {
                    follower.followPath(gateCycleCollected, 1.9, true);
                    setPathState(20);
                }
                break;*/

            case 20:
                if (!follower.isBusy()) {
                    autoManipulator.intake();

                    pathTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_COLLECT_SETTLE_TIME) {
                    autoManipulator.hold();
                    follower.followPath(gateCycleShoot, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    setPathState(23);
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(24);
                }
                break;
            case 24:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(25);
                }
                break;

            // =========================
            // Gate cycle, hold, collect, return to main shoot
            // =========================
            case 25:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(26);
                }
                break;

            case 26:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                   // follower.followPath(gateCycleCollect, true);
                    setPathState(28);
                }
                break;

          /*  case 27:
                if (!follower.isBusy()) {
                    follower.followPath(gateCycleCollected, 1.9, true);
                    setPathState(28);
                }
                break;*/

            case 28:
                if (!follower.isBusy()) {
                    autoManipulator.intake();

                    pathTimer.resetTimer();
                    setPathState(29);
                }
                break;

            case 29:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_COLLECT_SETTLE_TIME) {
                    autoManipulator.hold();
                    follower.followPath(gateCycleShoot, true);
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy()) {
                    setPathState(31);
                }
                break;

            case 31:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(32);
                }
                break;
            case 32:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(intakeFirstTriple, 1.0, true);
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromAFinal, true);
                    setPathState(34);
                }
                break;

            case 34:
                if (!follower.isBusy()) {
                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(36);
                }
                break;

            case 36:
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