// CloseGateBase.java
package org.firstinspires.ftc.teamcode.BaseAuto;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.AutoManipulator;
import org.firstinspires.ftc.teamcode.Helpers.FieldMirror;
import org.firstinspires.ftc.teamcode.Helpers.PoseHandoff;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

public abstract class CloseGateBase extends OpMode {

    protected abstract Alliance getAlliance();

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoManipulator autoManipulator;

    private LynxModule[] allHubs;
    private LynxModule controlHub;

    private double cachedVoltageComp = 1.0;
    private long lastVoltageReadMs = 0L;
    private static final long VOLTAGE_READ_INTERVAL_MS = 1000L;

    private int pathState;

    private static final double SHOOT_SETTLE_TIME = 0;
    private static final double GATE_COLLECT_SETTLE_TIME = 0;
    private static final double GATE_CYCLE_TIME = 1.7;
    private static final double SHOOTER_VELOCITY = 1590;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    private Pose startPose;
    public static Pose finalPose;

    private Pose IntakeA;
    private Pose IntakeACurveMid;
    private Pose IntakeB;
    private Pose IntakeBCurveMid;
    private Pose CollectedA;
    private Pose CollectedB;
    private Pose SidePushPtMID;
    private Pose SidePushPt;
    private Pose SHOOT_PRELOAD;
    private Pose SHOOT_CYCLE_1;
    private Pose SHOOT_CYCLE_1_MidPt;
    private Pose SHOOT_CYCLE_2;
    private Pose SHOOT_CYCLE_3;
    private Pose SHOOT_CYCLE_4;
    private Pose SHOOT_CYCLE_5;
    private Pose LEAVE_POINT;
    private Pose shootBMidPt;
    private Pose ActualGateCyclePt;
    private Pose gateCycleMid;

    private PathChain shootPreload;
    private PathChain intakeSecondTriple;
    private PathChain shootFromB;
    private PathChain gateCycleActually;
    private PathChain gateCycleShoot1;
    private PathChain gateCycleShoot2;
    private PathChain gateCycleShoot3;
    private PathChain intakeFirstTriple;
    private PathChain shootFromAFinal;
    private PathChain leavePath;

    private Pose p(double x, double y, double headingDeg) {
        return FieldMirror.pose(getAlliance(), x, y, headingDeg);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");

        initializeHubs();

        follower = Constants.createFollower(hardwareMap);

        buildPoses();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        autoManipulator = new AutoManipulator(intake, gate, telemetry);

        buildPaths();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void init_loop() {
        Pose pose = follower.getPose();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Waiting for Start");

        if (pose != null) {
            telemetry.addData("Robot X", pose.getX());
            telemetry.addData("Robot Y", pose.getY());
            telemetry.addData("Robot Heading", Math.toDegrees(pose.getHeading()));
        }

        autoManipulator.addTelemetry();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Started");
    }

    @Override
    public void loop() {
        prepareLoopTiming();

        follower.update();

        Pose pose = follower.getPose();
        if (pose == null) return;

        autoManipulator.update();

        shooter.setRobotY(pose.getY());
        shooter.setVelocity(SHOOTER_VELOCITY);
        // shooter.update(currentVoltageComp, loopDtSec);

        autonomousPathUpdate();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Voltage Comp", cachedVoltageComp);
        telemetry.addData("Shooter Avg Vel", shooter.getAverageVelocity());
        autoManipulator.addTelemetry();
    }

    @Override
    public void stop() {
        if (autoManipulator != null) {
            autoManipulator.stopAll();
        }

        if (shooter != null) {
            shooter.stop();
        }

        if (follower != null) {
            PoseHandoff.save(follower.getPose());
            finalPose = follower.getPose();
        }

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
    }

    private void initializeHubs() {
        List<LynxModule> hubsList = hardwareMap.getAll(LynxModule.class);
        allHubs = new LynxModule[hubsList.size()];

        for (int i = 0; i < hubsList.size(); i++) {
            allHubs[i] = hubsList.get(i);
            allHubs[i].setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

            if (allHubs[i].isParent()) {
                controlHub = allHubs[i];
            }
        }

        if (controlHub == null && allHubs.length > 0) {
            controlHub = allHubs[0];
        }

        if (controlHub != null) {
            cachedVoltageComp = getVoltageComp();
        }
    }

    private void prepareLoopTiming() {
        if (allHubs != null) {
            for (int i = 0; i < allHubs.length; i++) {
                allHubs[i].clearBulkCache();
            }
        }

        long nowMs = System.nanoTime() / 1_000_000L;

        if (controlHub != null && nowMs - lastVoltageReadMs > VOLTAGE_READ_INTERVAL_MS) {
            cachedVoltageComp = getVoltageComp();
            lastVoltageReadMs = nowMs;
        }
    }

    private double getVoltageComp() {
        if (controlHub == null) return cachedVoltageComp;

        double voltage = controlHub.getInputVoltage(VoltageUnit.VOLTS);

        if      (voltage < 8.0)  voltage = 8.0;
        else if (voltage > 14.0) voltage = 14.0;

        double rawComp = Math.pow(NOMINAL_VOLTAGE / voltage, VOLTAGE_COMP_POWER);

        if      (rawComp < 0.85) rawComp = 0.85;
        else if (rawComp > 1.45) rawComp = 1.45;

        return rawComp;
    }

    private void buildPoses() {
        startPose = p(18, -116, -90);

        IntakeA = p(38.5, -78, 0);
        IntakeACurveMid = p(55.5, -83, 0);

        IntakeB = p(38.5, -59, 0);
        IntakeBCurveMid = p(55.5, -59, 0);

        CollectedA = p(16, -78, 0);
        CollectedB = p(4, -58, 0);

        SidePushPtMID = p(12.5, -60, -120);
        SidePushPt = p(12.5, -66.39, -120);

        SHOOT_PRELOAD = p(56, -84, -45);

        SHOOT_CYCLE_1 = p(56, -84, -45);
        SHOOT_CYCLE_1_MidPt = p(42, -59, -45);

        SHOOT_CYCLE_2 = p(56, -84, -45);
        SHOOT_CYCLE_3 = p(56, -84, -45);
        SHOOT_CYCLE_4 = p(56, -84, -45);
        SHOOT_CYCLE_5 = p(56, -84, -45);

        LEAVE_POINT = p(42, -70, -45);

        shootBMidPt = p(42, -67, -90);
        ActualGateCyclePt = p(10.8, -59.7, 155.5);
        gateCycleMid = p(47.8, -71, -110);
    }

    private void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SHOOT_PRELOAD))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT_PRELOAD.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_PRELOAD, IntakeB, IntakeBCurveMid, CollectedB))
                .setConstantHeadingInterpolation(IntakeB.getHeading())
                .build();

        shootFromB = follower.pathBuilder()
                .addPath(new BezierCurve(CollectedB, SidePushPtMID, SidePushPt))
                .setConstantHeadingInterpolation(SidePushPt.getHeading())
                .addPath(new BezierCurve(SidePushPt, SHOOT_CYCLE_1_MidPt, SHOOT_CYCLE_1))
                .setLinearHeadingInterpolation(SidePushPt.getHeading(), SHOOT_CYCLE_1.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleActually = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_1, gateCycleMid, ActualGateCyclePt))
                .setLinearHeadingInterpolation(SHOOT_CYCLE_1.getHeading(), ActualGateCyclePt.getHeading())
                .build();

        gateCycleShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_2))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_2.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_3))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_3.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_4))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_4.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_4, IntakeA, IntakeACurveMid, CollectedA))
                .setConstantHeadingInterpolation(IntakeA.getHeading())
                .build();

        shootFromAFinal = follower.pathBuilder()
                .addPath(new BezierLine(CollectedA, SHOOT_CYCLE_5))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), SHOOT_CYCLE_5.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_CYCLE_5, LEAVE_POINT))
                .setConstantHeadingInterpolation(LEAVE_POINT.getHeading())
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
                if (!follower.isBusy()) setPathState(2);
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
                if (pathAlmostDone(SHOOT_CYCLE_2.getHeading(), HEADING_TOLERANCE_DEG)) {
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
                if (pathAlmostDone(SHOOT_CYCLE_3.getHeading(), HEADING_TOLERANCE_DEG)) {
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
                if (pathAlmostDone(SHOOT_CYCLE_4.getHeading(), HEADING_TOLERANCE_DEG)) {
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
                if (pathAlmostDone(SHOOT_CYCLE_5.getHeading(), HEADING_TOLERANCE_DEG)) {
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
                    autoManipulator.idle();
                    follower.followPath(leavePath, true);
                    setPathState(37);
                }
                break;

            case 37:
                if (!follower.isBusy()) setPathState(-1);
                break;

            case -1:
            default:
                break;
        }
    }

    private boolean pathAlmostDone(double targetHeadingRad, double headingToleranceDeg) {
        Pose pose = follower.getPose();
        if (pose == null) return false;

        double error = Math.atan2(
                Math.sin(pose.getHeading() - targetHeadingRad),
                Math.cos(pose.getHeading() - targetHeadingRad)
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