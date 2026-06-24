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
    private LynxModule shooterHub;

    private double cachedVoltageComp = 1.0;
    private long lastVoltageUpdateNs = 0L;
    private boolean wasVoltageFastModeLastLoop = false;

    private static final long VOLTAGE_IDLE_UPDATE_INTERVAL_NS = 250_000_000L;
    private static final long VOLTAGE_AIM_UPDATE_INTERVAL_NS  = 50_000_000L;

    private static final double DEFAULT_LOOP_DT_SEC = 0.02;
    private static final double MIN_LOOP_DT_SEC = 0.001;
    private static final double MAX_LOOP_DT_SEC = 0.1;
    private long lastLoopNs = 0L;
    private double loopDtSec = DEFAULT_LOOP_DT_SEC;

    private int pathState;

    private static final double SHOOT_SETTLE_TIME = 0;
    private static final double GATE_COLLECT_SETTLE_TIME = 0;
    private static final double GATE_CYCLE_TIME = 1.4;
    private static final double AFTER_INTAKE_SECOND_TRIPLE_WAIT_TIME = 0.2;
    private static final double NORMAL_SHOOTER_VELOCITY = 1580;
    private static final double PRELOAD_SHOOTER_VELOCITY = 1670;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    // Preload shoot-on-the-move timing
    private static final double PRELOAD_RELEASE_PARAM = 0.20;
    private static final double PRELOAD_SHOOT_REQUEST_PARAM = 0.45;
    private boolean preloadShootRequested = false;
    private boolean preloadShotStarted = false;

    // Final A shoot-on-the-move timing
    private static final double FINAL_A_RELEASE_PARAM = 0.2;
    private static final double FINAL_A_SHOOT_REQUEST_PARAM = 0.99;
    private boolean finalAShootRequested = false;
    private boolean finalAShotStarted = false;

    private Pose startPose;
    public static Pose finalPose;

    private Pose IntakeA;
    private Pose IntakeACurveMid;
    private Pose IntakeB;
    private Pose IntakeBCurveMid;
    private Pose CollectedA;
    private Pose CollectedB;
    private Pose SHOOT_PRELOAD;
    private Pose SHOOT_CYCLE_1;
    private Pose SHOOT_CYCLE_2;
    private Pose SHOOT_CYCLE_3;
    private Pose SHOOT_CYCLE_4;
    private Pose SHOOT_CYCLE_5;
    private Pose SHOOT_CYCLE_6;
    private Pose SHOOT_CYCLE_6_SHOOT_POINT;
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
    private PathChain gateCycleShoot4;
    private PathChain intakeFirstTriple;
    private PathChain shootFromAFinal;

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
        resetLoopTiming();

        lastVoltageUpdateNs = 0L;
        wasVoltageFastModeLastLoop = false;

        preloadShootRequested = false;
        preloadShotStarted = false;

        finalAShootRequested = false;
        finalAShotStarted = false;

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
        shooter.setVelocity(getShooterVelocityForCurrentState());

        final long nowNs = System.nanoTime();

        final boolean voltageFastMode = isAutoManipulatorShootingState();

        final boolean forceVoltageRefresh =
                lastVoltageUpdateNs == 0L || (voltageFastMode && !wasVoltageFastModeLastLoop);

        final double currentVoltageComp = getVoltageComp(
                nowNs,
                voltageFastMode,
                forceVoltageRefresh
        );

        wasVoltageFastModeLastLoop = voltageFastMode;

        shooter.update(currentVoltageComp, loopDtSec);

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
        telemetry.addData("Voltage Mode", voltageFastMode ? "FAST" : "IDLE");
        telemetry.addData("Loop dt", String.format(Locale.US, "%.1f ms", loopDtSec * 1000.0));
        telemetry.addData("Shooter Target", shooter.getTargetVelocity());
        telemetry.addData("Shooter Avg Vel", shooter.getAverageVelocity());
        telemetry.addData("Shooter Ready", shooter.isReadyToShoot());

        telemetry.addData("Preload Shot Requested", preloadShootRequested);
        telemetry.addData("Preload Shot Started", preloadShotStarted);

        telemetry.addData("Final A Shot Requested", finalAShootRequested);
        telemetry.addData("Final A Shot Started", finalAShotStarted);

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

            if (!allHubs[i].isParent()) {
                shooterHub = allHubs[i];
            }
        }

        if (shooterHub == null && allHubs.length > 0) {
            shooterHub = allHubs[0];
        }

        if (shooterHub != null) {
            cachedVoltageComp = getVoltageComp(System.nanoTime(), true, true);
        }
    }

    private void resetLoopTiming() {
        lastLoopNs = System.nanoTime();
        loopDtSec = DEFAULT_LOOP_DT_SEC;
    }

    private void prepareLoopTiming() {
        final long nowNs = System.nanoTime();

        if (lastLoopNs == 0L) {
            loopDtSec = DEFAULT_LOOP_DT_SEC;
        } else {
            loopDtSec = (nowNs - lastLoopNs) / 1_000_000_000.0;

            if (loopDtSec < MIN_LOOP_DT_SEC) {
                loopDtSec = MIN_LOOP_DT_SEC;
            } else if (loopDtSec > MAX_LOOP_DT_SEC) {
                loopDtSec = MAX_LOOP_DT_SEC;
            }
        }

        lastLoopNs = nowNs;

        if (allHubs != null) {
            for (int i = 0; i < allHubs.length; i++) {
                allHubs[i].clearBulkCache();
            }
        }
    }

    private double getVoltageComp(
            final long nowNs,
            final boolean fastMode,
            final boolean forceRefresh
    ) {
        if (shooterHub == null) return cachedVoltageComp;

        final long intervalNs =
                fastMode ? VOLTAGE_AIM_UPDATE_INTERVAL_NS : VOLTAGE_IDLE_UPDATE_INTERVAL_NS;

        if (!forceRefresh && nowNs - lastVoltageUpdateNs < intervalNs) {
            return cachedVoltageComp;
        }

        lastVoltageUpdateNs = nowNs;

        double voltage = shooterHub.getInputVoltage(VoltageUnit.VOLTS);

        if (voltage <= 0.0 || Double.isNaN(voltage)) {
            return cachedVoltageComp;
        }

        if      (voltage < 8.0)  voltage = 8.0;
        else if (voltage > 14.0) voltage = 14.0;

        final double ratio = NOMINAL_VOLTAGE / voltage;

        double rawComp;
        if (VOLTAGE_COMP_POWER == 1.0) {
            rawComp = ratio;
        } else if (VOLTAGE_COMP_POWER == 0.0) {
            rawComp = 1.0;
        } else {
            rawComp = Math.pow(ratio, VOLTAGE_COMP_POWER);
        }

        if      (rawComp < 0.85) rawComp = 0.85;
        else if (rawComp > 1.45) rawComp = 1.45;

        cachedVoltageComp = rawComp;
        return cachedVoltageComp;
    }

    private void buildPoses() {
        startPose = p(17.57, 117.2, -90);

        IntakeA = p(38.5, 82.54, 180);
        IntakeACurveMid = p(55.5, 83, 180);

        IntakeB = p(38.5, 62.4, 180);
        IntakeBCurveMid = p(55.5, 62.4, 180);

        CollectedA = p(16, 82.54, 180);
        CollectedB = p(16.5, 62.4, 180);

        SHOOT_PRELOAD = p(56, 84, -46);

        SHOOT_CYCLE_1 = p(56, 84, -50);
        SHOOT_CYCLE_2 = p(56, 84, -50);
        SHOOT_CYCLE_3 = p(56, 84, -50);
        SHOOT_CYCLE_4 = p(56, 84, -50);
        SHOOT_CYCLE_5 = p(56, 84, -50);

        // Final A shoot-on-the-move:
        // Robot turns to -35 degrees before the final point, then keeps that heading.
        SHOOT_CYCLE_6_SHOOT_POINT = p(38.0, 96.0, -48);

        // New final point. Auto ends here after the final shot.
        SHOOT_CYCLE_6 = p(55.3, 101.9, -35);

        shootBMidPt = p(42, 67, -90);
        ActualGateCyclePt = p(10.8, 60, 150);
        gateCycleMid = p(47.8, 71, 110);
    }

    private void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SHOOT_PRELOAD))
                .setConstantHeadingInterpolation(SHOOT_PRELOAD.getHeading())
                .addParametricCallback(PRELOAD_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .addParametricCallback(PRELOAD_SHOOT_REQUEST_PARAM, () -> preloadShootRequested = true)
                .build();

        intakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_PRELOAD, IntakeB, IntakeBCurveMid, CollectedB))
                .setConstantHeadingInterpolation(IntakeB.getHeading())
                .build();

        shootFromB = follower.pathBuilder()
                // Go directly from collected B to the shooting point; no side-push path.
                .addPath(new BezierLine(CollectedB, SHOOT_CYCLE_1))
                .setLinearHeadingInterpolation(CollectedB.getHeading(), SHOOT_CYCLE_1.getHeading())
                .addParametricCallback(0.75, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleActually = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_1, gateCycleMid, ActualGateCyclePt))
                .setLinearHeadingInterpolation(SHOOT_CYCLE_1.getHeading(), ActualGateCyclePt.getHeading())
                .build();

        gateCycleShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_2))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_2.getHeading())
                .addParametricCallback(0.75, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_3))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_3.getHeading())
                .addParametricCallback(0.75, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_4))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_4.getHeading())
                .addParametricCallback(0.75, () -> autoManipulator.releaseForShot())
                .build();

        gateCycleShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(ActualGateCyclePt, shootBMidPt, SHOOT_CYCLE_5))
                .setLinearHeadingInterpolation(ActualGateCyclePt.getHeading(), SHOOT_CYCLE_5.getHeading())
                .addParametricCallback(0.75, () -> autoManipulator.releaseForShot())
                .build();

        intakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_CYCLE_5, IntakeA, IntakeACurveMid, CollectedA))
                .setConstantHeadingInterpolation(IntakeA.getHeading())
                .build();

        shootFromAFinal = follower.pathBuilder()
                // Part 1: leave CollectedA and rotate toward the final shooting heading.
                .addPath(new BezierLine(CollectedA, SHOOT_CYCLE_6_SHOOT_POINT))
                .setLinearHeadingInterpolation(CollectedA.getHeading(), SHOOT_CYCLE_6_SHOOT_POINT.getHeading())

                // Part 2: keep -35 degrees while moving to the new SHOOT_CYCLE_6 point.
                .addPath(new BezierLine(SHOOT_CYCLE_6_SHOOT_POINT, SHOOT_CYCLE_6))
                .setLinearHeadingInterpolation(SHOOT_CYCLE_6_SHOOT_POINT.getHeading(), SHOOT_CYCLE_6.getHeading())

                // Release before the shot, then request the shot around 60% of the full path.
                .addParametricCallback(FINAL_A_RELEASE_PARAM, () -> autoManipulator.releaseForShot())
                .addParametricCallback(FINAL_A_SHOOT_REQUEST_PARAM, () -> finalAShootRequested = true)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                preloadShootRequested = false;
                preloadShotStarted = false;

                autoManipulator.hold();
                follower.followPath(shootPreload, true);
                setPathState(1);
                break;

            case 1:
                if (preloadShootRequested && !preloadShotStarted && shooter.isReadyToShoot()) {
                    autoManipulator.shoot();
                    preloadShotStarted = true;
                }

                if (!follower.isBusy() && preloadShotStarted && autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(intakeSecondTriple, 1.0, true);
                    setPathState(5);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= AFTER_INTAKE_SECOND_TRIPLE_WAIT_TIME) {
                    autoManipulator.hold();
                    follower.followPath(shootFromB, true);
                    setPathState(6);
                }
                break;

            case 4:
                break;

            case 5:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    setPathState(2);
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
                    follower.followPath(gateCycleActually, true);
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(34);
                }
                break;

            case 34:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_CYCLE_TIME) {
                    autoManipulator.intake();
                    setPathState(36);
                }
                break;

            case 36:
                if (!follower.isBusy()) {
                    autoManipulator.intake();
                    pathTimer.resetTimer();
                    setPathState(37);
                }
                break;

            case 37:
                if (pathTimer.getElapsedTimeSeconds() >= GATE_COLLECT_SETTLE_TIME) {
                    autoManipulator.hold();
                    follower.followPath(gateCycleShoot4, true);
                    setPathState(38);
                }
                break;

            case 38:
                if (pathAlmostDone(SHOOT_CYCLE_5.getHeading(), HEADING_TOLERANCE_DEG)) {
                    setPathState(39);
                }
                break;

            case 39:
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SETTLE_TIME) {
                    autoManipulator.shoot();
                    setPathState(40);
                }
                break;

            case 40:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(intakeFirstTriple, 1.0, true);
                    setPathState(41);
                }
                break;

            case 41:
                if (!follower.isBusy()) {
                    finalAShootRequested = false;
                    finalAShotStarted = false;

                    autoManipulator.hold();
                    follower.followPath(shootFromAFinal, true);
                    setPathState(42);
                }
                break;

            case 42:
                // Final A shoot-on-the-move.
                // The path requests the shot around 60%.
                // The shot only feeds after shooter.isReadyToShoot().
                if (finalAShootRequested && !finalAShotStarted && shooter.isReadyToShoot()) {
                    autoManipulator.shoot();
                    finalAShotStarted = true;
                }

                // No leave path anymore.
                // Auto ends at SHOOT_CYCLE_6 after final shot completes.
                if (!follower.isBusy() && finalAShotStarted && autoManipulator.isShootComplete()) {
                    autoManipulator.idle();
                    setPathState(-1);
                }
                break;

            case 43:
                break;

            case -1:
            default:
                break;
        }
    }

    private boolean isAutoManipulatorShootingState() {
        switch (pathState) {
            case 1:
            case 8:
            case 16:
            case 24:
            case 32:
            case 40:
            case 42:
                return true;

            default:
                return false;
        }
    }
    private double getShooterVelocityForCurrentState() {
        switch (pathState) {
            case 0:
            case 1:
                return PRELOAD_SHOOTER_VELOCITY;

            default:
                return NORMAL_SHOOTER_VELOCITY;
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