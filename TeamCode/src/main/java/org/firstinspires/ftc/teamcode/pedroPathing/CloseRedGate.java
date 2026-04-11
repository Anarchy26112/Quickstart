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

@Autonomous(name = "CloseRedGateSolo", group = "Auto")
public class CloseRedGate extends OpMode {

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
    private static final double GATE_CYCLE_TIME = 1.4;
    private static final double SHOOTER_VELOCITY = 1520;

    // =========================
    // Starting pose + mirrored field points
    // =========================
    private final Pose startPose = new Pose(45, 124, Math.toRadians(-140));

    public static Pose finalPose;

    private final Pose IntakeA = new Pose(25, 75, Math.toRadians(0));
    private final Pose IntakeACurveMid = new Pose(8, 75, Math.toRadians(0));

    private final Pose IntakeB = new Pose(24, 51, Math.toRadians(0));
    private final Pose IntakeBCurveMid = new Pose(7, 51, Math.toRadians(0));

    private final Pose IntakeC = new Pose(25, 27, Math.toRadians(0));
    private final Pose IntakeCCurveMid = new Pose(8, 27, Math.toRadians(0));

    private final Pose CollectedA = new Pose(54.5, 75, Math.toRadians(0));
    private final Pose CollectedB = new Pose(62, 51, Math.toRadians(0));
    private final Pose CollectedC = new Pose(62, 27, Math.toRadians(0));

    // =========================
    // Separate shooting poses
    // Mirrored from blue: y -> -y, heading -> -heading
    // =========================

    // Preload shot
    private final Pose SHOOT_PRELOAD = new Pose(20, 80, Math.toRadians(-135));

    // Standard cycle shot after collecting B
    private final Pose SHOOT_CYCLE_1 = new Pose(20, 80, Math.toRadians(-127));

    // Gate cycle return shot #1
    private final Pose SHOOT_CYCLE_2 = new Pose(20, 80, Math.toRadians(-135));

    // Gate cycle return shot #2
    private final Pose SHOOT_CYCLE_3 = new Pose(20, 80, Math.toRadians(-135));

    // Gate cycle return shot #3
    private final Pose SHOOT_CYCLE_4 = new Pose(20, 80, Math.toRadians(-135));

    // Final shot after collecting A
    private final Pose SHOOT_FINAL = new Pose(14, 95, Math.toRadians(-137.5));

    private final Pose shootBMidPt = new Pose(18, 58, Math.toRadians(-90));
    private final Pose PushCycle = new Pose(55.55, 59.5, Math.toRadians(0));
    private final Pose ActualGateCyclePt = new Pose(63, 54.5, Math.toRadians(-325.65));

    private final Pose gateCycleMid = new Pose(24.2, 61, Math.toRadians(-70));
    private final Pose CycleCollect = new Pose(60.5, 43.5, Math.toRadians(67));
    private final Pose CycleCollected = new Pose(60.5, 47, Math.toRadians(67));

    // =========================
    // Paths
    // =========================
    private PathChain shootPreload;

    private PathChain intakeSecondTriple;
    private PathChain shootFromB;

    private PathChain gateCycleActually;

    private PathChain gateCycleShoot1;
    private PathChain gateCycleShoot2;
    private PathChain gateCycleShoot3;

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
        setPathState(0); // 0 to run actual auto, -1 for telemetry only

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autoManipulator.update();

        shooter.setVelocity(SHOOTER_VELOCITY);
        shooter.update(System.nanoTime());

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter Avg Vel", shooter.getAverageVelocity());
        telemetry.addData("Shooter Target", shooter.getTargetVelocity());
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
        // =========================
        // Preload shot
        // =========================
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SHOOT_PRELOAD))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT_PRELOAD.getHeading())
                .build();

        // =========================
        // Collect B, then shoot from cycle 1 point
        // =========================
        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_PRELOAD, IntakeB, IntakeBCurveMid, CollectedB))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shootFromB = follower.pathBuilder()
                .addPath(new BezierCurve(CollectedB, shootBMidPt, SHOOT_CYCLE_1))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), SHOOT_CYCLE_1.getHeading())
                .build();

        // =========================
        // Gate cycle out
        // =========================
        gateCycleActually = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_1, gateCycleMid, ActualGateCyclePt))
                .setLinearHeadingInterpolation(SHOOT_CYCLE_1.getHeading(), ActualGateCyclePt.getHeading())
                .build();

        // =========================
        // Gate return shots
        // =========================
        gateCycleShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(CycleCollected, shootBMidPt, SHOOT_CYCLE_2))
                .setLinearHeadingInterpolation(CycleCollected.getHeading(), SHOOT_CYCLE_2.getHeading())
                .build();

        gateCycleShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(CycleCollected, shootBMidPt, SHOOT_CYCLE_3))
                .setLinearHeadingInterpolation(CycleCollected.getHeading(), SHOOT_CYCLE_3.getHeading())
                .build();

        gateCycleShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(CycleCollected, shootBMidPt, SHOOT_CYCLE_4))
                .setLinearHeadingInterpolation(CycleCollected.getHeading(), SHOOT_CYCLE_4.getHeading())
                .build();

        // =========================
        // Final intake and final shot
        // =========================
        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_4, IntakeA, IntakeACurveMid, CollectedA))
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
                    setPathState(4);
                }
                break;

            // =========================
            // Collect second, shoot cycle 1
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
            // Gate cycle -> return -> shoot cycle 2
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
                    setPathState(12);
                }
                break;

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
                    follower.followPath(gateCycleShoot1, true);
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

            // =========================
            // Gate cycle -> return -> shoot cycle 3
            // =========================
            case 16:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(17);
                }
                break;

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
                    setPathState(20);
                }
                break;

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
                    follower.followPath(gateCycleShoot2, true);
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

            // =========================
            // Gate cycle -> return -> shoot cycle 4
            // =========================
            case 24:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(25);
                }
                break;

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
                    setPathState(28);
                }
                break;

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
                    follower.followPath(gateCycleShoot3, true);
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

            // =========================
            // Final intake and final shot
            // =========================
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