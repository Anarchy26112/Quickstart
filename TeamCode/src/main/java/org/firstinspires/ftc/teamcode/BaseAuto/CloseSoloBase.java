package org.firstinspires.ftc.teamcode.BaseAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.AutoManipulator;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;
import org.firstinspires.ftc.teamcode.Helpers.PoseHandoff;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

public abstract class CloseSoloBase extends OpMode {

    protected abstract Alliance getAlliance();

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
    private static final double GATE_CYCLE_TIME = 1.7;
    private static final double SHOOTER_VELOCITY = 1550;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    // Start with 0.90 if you want safer tuning, then move earlier later
    private static final double SHOT_RELEASE_PARAM = 0.9;

    // =========================
    // Poses
    // Defined once in BLUE coordinates,
    // mirrored automatically for RED
    // =========================
    private Pose startPose;

    public static Pose finalPose;

    private Pose IntakeA;
    private Pose IntakeACurveMid;

    private Pose IntakeB;
    private Pose IntakeBCurveMid;

    private Pose IntakeC;
    private Pose IntakeCCurveMid;

    private Pose CollectedA;
    private Pose CollectedB;
    private Pose CollectedC;

    private Pose SHOOT_PRELOAD;
    private Pose SHOOT_CYCLE_1;
    private Pose SHOOT_CYCLE_2;
    private Pose SHOOT_CYCLE_3;
    private Pose SHOOT_CYCLE_4;
    private Pose SHOOT_FINAL;

    private Pose shootBMidPt;
    private Pose ActualGateCyclePt;
    private Pose gateCycleMid;

    // =========================
    // Paths
    // =========================
    private PathChain shootPreload;

    private PathChain intakeSecondTriple;
    private PathChain shootFromB;

    private PathChain gateCycleActually;
    private PathChain gateCycleShoot1;
    private PathChain gateCycleShoot2;

    private PathChain intakeThirdTriple;
    private PathChain shootFromC;

    private PathChain intakeFirstTriple;
    private PathChain shootFromAFinal;

    private Pose p(double x, double y, double headingDeg) {
        return FieldMirror.pose(getAlliance(), x, y, headingDeg);
    }

    private double h(double headingDeg) {
        return FieldMirror.headingRad(getAlliance(), headingDeg);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);

        buildPoses();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        autoManipulator = new AutoManipulator(intake, gate, telemetry);

        buildPaths();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", getAlliance());
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

        telemetry.addData("Alliance", getAlliance());
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

        telemetry.addData("Alliance", getAlliance());
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

    private void buildPoses() {
        startPose = p(45, -124, 140);

        IntakeA = p(25, -75, 0);
        IntakeACurveMid = p(8, -75, 0);

        IntakeB = p(25, -51, 0);
        IntakeBCurveMid = p(8, -51, 0);

        IntakeC = p(25, -27, 0);
        IntakeCCurveMid = p(8, -27, 0);

        CollectedA = p(54.5, -75, 0);
        CollectedB = p(62, -51, 0);
        CollectedC = p(62, -27, 0);

        // Multi-shot geometry
        // Start with these, then tune on field
        SHOOT_PRELOAD = p(20, -80, 135);
        SHOOT_CYCLE_1 = p(20, -82, 127);
        SHOOT_CYCLE_2 = p(20, -84, 131);
        SHOOT_CYCLE_3 = p(20, -86, 134);
        SHOOT_CYCLE_4 = p(18, -90, 138);
        SHOOT_FINAL   = p(17, -100, 142);

        shootBMidPt = p(18, -58, 90);
        ActualGateCyclePt = p(63, -55.15, 332);
        gateCycleMid = p(24.2, -61, 70);
    }

    public void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SHOOT_PRELOAD))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT_PRELOAD.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_PRELOAD, IntakeB, IntakeBCurveMid, CollectedB))
                .setConstantHeadingInterpolation(h(0))
                .build();

        shootFromB = follower.pathBuilder()
                .addPath(new BezierCurve(CollectedB, shootBMidPt, SHOOT_CYCLE_1))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), SHOOT_CYCLE_1.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleActually = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_1, gateCycleMid, ActualGateCyclePt))
                .setLinearHeadingInterpolation(SHOOT_CYCLE_1.getHeading(), ActualGateCyclePt.getHeading())
                .build();

        gateCycleShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_2))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_2.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_3))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_3.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();

        intakeThirdTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_3, IntakeC, IntakeCCurveMid, CollectedC))
                .setConstantHeadingInterpolation(h(0))
                .build();

        shootFromC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedC, SHOOT_CYCLE_4))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), SHOOT_CYCLE_4.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();

        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_4, IntakeA, IntakeACurveMid, CollectedA))
                .setConstantHeadingInterpolation(h(0))
                .build();

        shootFromAFinal = follower.pathBuilder()
                .addPath(new BezierLine(CollectedA, SHOOT_FINAL))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), SHOOT_FINAL.getHeading())
                .addParametricCallback(SHOT_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                autoManipulator.hold();
                follower.followPath(shootPreload, true);
                setPathState(1);
                break;

            case 1:
                if (pathAlmostDone(SHOOT_PRELOAD.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(4);
                }
                break;

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
                if (pathAlmostDone(SHOOT_CYCLE_1.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(8);
                }
                break;

            // =========================
            // Gate cycle 1
            // =========================
            case 8:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(9);
                }
                break;

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
                    setPathState(11);
                }
                break;

            case 11:
                autoManipulator.hold();
                follower.followPath(gateCycleShoot1, true);
                setPathState(12);
                break;

            case 12:
                if (pathAlmostDone(SHOOT_CYCLE_2.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(14);
                }
                break;

            // =========================
            // Gate cycle 2
            // =========================
            case 14:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(gateCycleActually, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                    setPathState(17);
                }
                break;

            case 17:
                autoManipulator.hold();
                follower.followPath(gateCycleShoot2, true);
                setPathState(18);
                break;

            case 18:
                if (pathAlmostDone(SHOOT_CYCLE_3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(20);
                }
                break;

            // =========================
            // Continue solo auto
            // =========================
            case 20:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(intakeThirdTriple, 1.0, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromC, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (pathAlmostDone(SHOOT_CYCLE_4.getHeading(), HEADING_TOLERANCE_DEG)) {
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
                    follower.followPath(intakeFirstTriple, 1.0, true);
                    setPathState(25);
                }
                break;

            case 25:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(shootFromAFinal, true);
                    setPathState(26);
                }
                break;

            case 26:
                if (pathAlmostDone(SHOOT_FINAL.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(27);
                }
                break;

            case 27:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(28);
                }
                break;

            case 28:
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

    private boolean pathAlmostDone(double targetHeadingRad, double headingToleranceDeg) {
        double error = Math.atan2(
                Math.sin(follower.getPose().getHeading() - targetHeadingRad),
                Math.cos(follower.getPose().getHeading() - targetHeadingRad)
        );
        return Math.abs(Math.toDegrees(error)) < headingToleranceDeg;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected Follower getFollower() {
        return follower;
    }
}