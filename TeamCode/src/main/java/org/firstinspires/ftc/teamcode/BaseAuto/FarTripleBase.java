package org.firstinspires.ftc.teamcode.BaseAuto;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.pedropathing.follower.Follower;
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

public abstract class FarTripleBase extends OpMode {

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

    private static final long VOLTAGE_IDLE_UPDATE_INTERVAL_NS = 500_000_000L;
    private static final long VOLTAGE_AIM_UPDATE_INTERVAL_NS  = 100_000_000L;

    private static final double DEFAULT_LOOP_DT_SEC = 0.02;
    private static final double MIN_LOOP_DT_SEC = 0.001;
    private static final double MAX_LOOP_DT_SEC = 0.1;
    private long lastLoopNs = 0L;
    private double loopDtSec = DEFAULT_LOOP_DT_SEC;

    private int pathState;
    private int scatterCycleIndex = 0;
    private int collectedShakePhase = -1;

    private double intakeAToCollectedTimeSec = Double.NaN;
    private double intakeBToCollectedTimeSec = Double.NaN;
    private double intakeSegmentStartSec = 0.0;
    private boolean timingIntakeSegment = false;
    private boolean intakeSegmentTimerStarted = false;
    private boolean watchIntakeSegmentTimer = false;
    private int watchedScatterChoice = -1;

    private static final double INTAKE_A_TO_COLLECTED_TIMEOUT_SEC = 0.85;
    private static final double INTAKE_B_TO_COLLECTED_TIMEOUT_SEC = 1.19;

    private static final int SCATTER_CYCLE_COUNT = 6;
    private static final double COLLECTED_SCATTER_WAIT_SEC = 0.5;
    private static final double COLLECTED_SCATTER_SHAKE_DEG = 22.5;
    private static final double COLLECTED_SCATTER_SHAKE_LEFT_END_SEC = COLLECTED_SCATTER_WAIT_SEC / 3.0;
    private static final double COLLECTED_SCATTER_SHAKE_RIGHT_END_SEC = 2.0 * COLLECTED_SCATTER_WAIT_SEC / 3.0;

    public static int[] scatterPlan = {0, 0, 1, 0, 1, 0};
    public static Pose finalPose;

    private static final double SHOOTER_VELOCITY = 1920;
    private static final double PRELOAD_SHOOT_DELAY = 0.0;


    private Pose startPose;

    private Pose IntakeScatterA;
    private Pose CollectedScatterA;

    private Pose IntakeC;
    private Pose CollectedC;

    private Pose IntakeScatterB;
    private Pose CollectedScatterB;

    private Pose IntakeScatterC;
    private Pose CollectedScatterC;

    private Pose ShootPreloadPoint;
    private Pose ShootAfterTripleC;
    private Pose ShootScatterA;
    private Pose ShootScatterB;


    private PathChain ShootPreload;
    private PathChain ThirdFull;

    private PathChain scatterAReturnToShoot;
    private PathChain scatterBReturnToShoot;

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

        autoManipulator.setIntakePower(1.0, 0.5);
        autoManipulator.setHoldingPower(0.0, 0.0);
        autoManipulator.setShootingFeedPower(1.0, 1.0);

        buildPaths();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Ready");
        telemetry.addData("Scatter Plan", getScatterPlanString());
    }

    @Override
    public void init_loop() {
        Pose pose = follower.getPose();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Scatter Plan", getScatterPlanString());

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

        scatterCycleIndex = 0;
        resetIntakeSegmentTimers();
        resetCollectedShake();

        lastVoltageUpdateNs = 0L;
        wasVoltageFastModeLastLoop = false;

        setPathState(0);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Started");
        telemetry.addData("Scatter Plan", getScatterPlanString());
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

        updateIntakeSegmentTiming();
        autonomousPathUpdate();

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
            Pose pose = follower.getPose();
            PoseHandoff.save(pose);
            finalPose = pose;
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
        startPose = p(55.8, 8.1, -90);

        IntakeScatterA = p(20.5, 7, 180);
        CollectedScatterA = p(12, 7, 180);

        IntakeC = p(44, 35, 180);
        CollectedC = p(19, 35, 180);

        IntakeScatterB = p(27, 27, 180);
        CollectedScatterB = p(12, 27, 180);

        IntakeScatterC = p(20, 40.7, 180);
        CollectedScatterC = p(15, 40.7, 180);

        // Four independent shoot points. They start with the old Shoot3 tuning
        // so the auto behaves the same until each point is retuned.
        ShootPreloadPoint = p(55.1, 13.5, -68.6);
        ShootAfterTripleC = p(55.1, 13.5, -72.3);
        ShootScatterA = p(55.1, 13.5, -73.4);
        ShootScatterB = p(55.1, 13.5, -71.6);

    }

    private void buildPaths() {
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPreloadPoint))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPreloadPoint.getHeading())
                .addParametricCallback(0.7, () -> autoManipulator.releaseForShot())
                .build();

        ThirdFull = follower.pathBuilder()
                .addPath(new BezierLine(ShootPreloadPoint, IntakeC))
                .setLinearHeadingInterpolation(ShootPreloadPoint.getHeading(), IntakeC.getHeading())

                .addPath(new BezierLine(IntakeC, CollectedC))
                .setConstantHeadingInterpolation(IntakeC.getHeading())

                .addPath(new BezierLine(CollectedC, ShootAfterTripleC))
                .setLinearHeadingInterpolation(headingFrom(ShootAfterTripleC, CollectedC), ShootAfterTripleC.getHeading())
                .addParametricCallback(0.5, () -> autoManipulator.hold())

                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        scatterAReturnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(CollectedScatterA, ShootScatterA))
                .setLinearHeadingInterpolation(CollectedScatterA.getHeading(), ShootScatterA.getHeading())
                .addParametricCallback(0.5, () -> autoManipulator.hold())

                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        scatterBReturnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(CollectedScatterB, ShootScatterB))
                .setLinearHeadingInterpolation(headingFrom(ShootScatterB, CollectedScatterB), ShootScatterB.getHeading())
                .addParametricCallback(0.5, () -> autoManipulator.hold())

                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();
    }

    private PathChain buildScatterAToCollected(Pose shootStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())
                .build();
    }

    private PathChain buildScatterBToCollected(Pose shootStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterB))
                .setLinearHeadingInterpolation(shootStart.getHeading(), headingFrom(shootStart, IntakeScatterB))

                .addPath(new BezierLine(IntakeScatterB, CollectedScatterB))
                .setConstantHeadingInterpolation(IntakeScatterB.getHeading())
                .build();
    }

    private PathChain buildScatterCFull(Pose shootStart) {
        Pose shootTarget = getShootPoseForScatterChoice(2);

        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterC))
                .setLinearHeadingInterpolation(shootStart.getHeading(), headingFrom(shootStart, IntakeScatterC))

                .addPath(new BezierLine(IntakeScatterC, CollectedScatterC))
                .setConstantHeadingInterpolation(IntakeScatterC.getHeading())

                .addPath(new BezierLine(CollectedScatterC, CollectedScatterB))
                .setConstantHeadingInterpolation(headingFrom(CollectedScatterC, CollectedScatterB))
                .addParametricCallback(0.03, () -> autoManipulator.hold())

                .addPath(new BezierLine(CollectedScatterB, shootTarget))
                .setLinearHeadingInterpolation(headingFrom(shootTarget, CollectedScatterB), shootTarget.getHeading())

                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();
    }


    private Pose getCurrentPoseOr(Pose fallback) {
        Pose current = follower.getPose();
        return current != null ? current : fallback;
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
                    follower.followPath(ThirdFull, 1.0, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(4);
                }
                break;

            case 4:
                if (autoManipulator.isShootComplete()) {
                    scatterCycleIndex = 0;
                    startNextScatterOrLeave();
                }
                break;

            case 5:
                /*
                 * For A/B, the first scatter path ends at CollectedScatterA/B.
                 *
                 * IMPORTANT:
                 * Do NOT call autoManipulator.hold() here.
                 * The robot is still in INTAKING state from startNextScatterOrLeave(),
                 * so intake and transfer keep spinning at 100% during the 0.5 sec wait.
                 */
                if (!follower.isBusy()) {
                    if (currentScatterNeedsCollectedWait()) {
                        resetCollectedShake();
                        setPathState(55);
                    } else {
                        autoManipulator.shoot();
                        setPathState(6);
                    }
                }
                break;

            case 55:
                /*
                 * Shake at CollectedScatterA/B while intake keeps spinning:
                 *   0.00-0.17 sec = 15 deg left of the wall-facing heading
                 *   0.17-0.33 sec = 15 deg right of the wall-facing heading
                 *                  (30 deg right from the left-shake target)
                 *   0.33-0.50 sec = back to the original wall-facing heading
                 *
                 * After the 0.5 sec shake/wait is over, then hold:
                 * gate blocked, intake stopped, transfer stopped.
                 */
                updateCollectedShakeHold();

                if (pathTimer.getElapsedTimeSeconds() >= COLLECTED_SCATTER_WAIT_SEC) {
                    cancelIntakeSegmentTimer();
                    resetCollectedShake();
                    autoManipulator.hold();
                    follower.followPath(getScatterReturnToShoot(scatterCycleIndex), 1.0, true);
                    setPathState(56);
                }
                break;

            case 56:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(6);
                }
                break;

            case 6:
                if (autoManipulator.isShootComplete()) {
                    scatterCycleIndex++;
                    startNextScatterOrLeave();
                }
                break;


            case -1:
            default:
                autoManipulator.idle();
                break;
        }
    }


    private void resetIntakeSegmentTimers() {
        intakeAToCollectedTimeSec = Double.NaN;
        intakeBToCollectedTimeSec = Double.NaN;
        cancelIntakeSegmentTimer();
    }

    private void resetCollectedShake() {
        collectedShakePhase = -1;
    }

    private void updateCollectedShakeHold() {
        int choice = getScatterChoiceForCycle(scatterCycleIndex);

        if (choice != 0 && choice != 1) {
            return;
        }

        double elapsedSec = pathTimer.getElapsedTimeSeconds();
        int phase;

        if (elapsedSec < COLLECTED_SCATTER_SHAKE_LEFT_END_SEC) {
            phase = 0;
        } else if (elapsedSec < COLLECTED_SCATTER_SHAKE_RIGHT_END_SEC) {
            phase = 1;
        } else {
            phase = 2;
        }

        if (phase == collectedShakePhase) {
            return;
        }

        Pose current = follower.getPose();

        if (current == null) {
            return;
        }

        double targetHeading = getCollectedShakeHeading(choice, phase);
        follower.holdPoint(new Pose(current.getX(), current.getY(), targetHeading), false);
        collectedShakePhase = phase;
    }

    private double getCollectedShakeHeading(int scatterChoice, int phase) {
        double wallFacingHeading = getCollectedWallFacingHeading(scatterChoice);
        double shakeRadians = Math.toRadians(COLLECTED_SCATTER_SHAKE_DEG);

        switch (phase) {
            case 0:
                return wallFacingHeading + shakeRadians;

            case 1:
                return wallFacingHeading - shakeRadians;

            case 2:
            default:
                return wallFacingHeading;
        }
    }

    private double getCollectedWallFacingHeading(int scatterChoice) {
        if (scatterChoice == 0) {
            return CollectedScatterA.getHeading();
        }

        return CollectedScatterB.getHeading();
    }

    private void armIntakeSegmentTimerForCycle() {
        watchedScatterChoice = getScatterChoiceForCycle(scatterCycleIndex);
        watchIntakeSegmentTimer = watchedScatterChoice == 0 || watchedScatterChoice == 1;
        timingIntakeSegment = false;
        intakeSegmentTimerStarted = false;
        intakeSegmentStartSec = 0.0;
    }

    private void cancelIntakeSegmentTimer() {
        watchIntakeSegmentTimer = false;
        timingIntakeSegment = false;
        intakeSegmentTimerStarted = false;
        intakeSegmentStartSec = 0.0;
        watchedScatterChoice = -1;
    }

    private void updateIntakeSegmentTiming() {
        if (!watchIntakeSegmentTimer || (pathState != 5 && pathState != 55)) {
            return;
        }

        double nowSec = opmodeTimer.getElapsedTimeSeconds();

        /*
         * The dynamic Scatter A/B intake paths both have two paths:
         *   path 0 = current shoot point -> IntakeScatterA/B
         *   path 1 = IntakeScatterA/B -> CollectedScatterA/B
         *
         * Start timing only when path 1 begins. The same timer keeps running
         * through pathState 55, so the 0.85 sec A timeout and 1.16 sec B
         * timeout include BOTH:
         *   1) IntakeScatterA/B -> CollectedScatterA/B travel
         *   2) the 0.5 sec collected shake/wait
         */
        boolean onIntakeToCollectedSegment =
                pathState == 5 && follower.isBusy() && follower.getCurrentPathNumber() >= 1;

        if (!timingIntakeSegment && !intakeSegmentTimerStarted && onIntakeToCollectedSegment) {
            timingIntakeSegment = true;
            intakeSegmentTimerStarted = true;
            intakeSegmentStartSec = nowSec;
        }

        if (!timingIntakeSegment) {
            return;
        }

        double elapsedSec = nowSec - intakeSegmentStartSec;

        if (watchedScatterChoice == 0) {
            intakeAToCollectedTimeSec = elapsedSec;
        } else if (watchedScatterChoice == 1) {
            intakeBToCollectedTimeSec = elapsedSec;
        }

        if (elapsedSec >= getIntakeSegmentTimeoutSec(watchedScatterChoice)) {
            timeoutBackToShoot();
        }
    }

    private double getIntakeSegmentTimeoutSec(int scatterChoice) {
        if (scatterChoice == 0) {
            return INTAKE_A_TO_COLLECTED_TIMEOUT_SEC;
        }

        return INTAKE_B_TO_COLLECTED_TIMEOUT_SEC;
    }

    private String formatTimeSec(double timeSec) {
        if (Double.isNaN(timeSec)) {
            return "--";
        }

        return String.format(Locale.US, "%.2f sec", timeSec);
    }

    private void startNextScatterOrLeave() {
        resetCollectedShake();

        if (scatterCycleIndex < SCATTER_CYCLE_COUNT) {
            autoManipulator.intake();
            armIntakeSegmentTimerForCycle();
            follower.followPath(getScatterPathToRun(scatterCycleIndex), 1.0, true);
            setPathState(5);
        } else {
            // No final leave path: stay at the last shooting position after the final shot.
            cancelIntakeSegmentTimer();
            autoManipulator.idle();
            setPathState(-1);
        }
    }

    private void timeoutBackToShoot() {
        int scatterChoice = watchedScatterChoice >= 0
                ? watchedScatterChoice
                : getScatterChoiceForCycle(scatterCycleIndex);

        cancelIntakeSegmentTimer();
        resetCollectedShake();
        autoManipulator.hold();
        follower.followPath(returnToShootFromCurrent(scatterChoice), 1.0, true);
        setPathState(56);
    }

    private PathChain returnToShootFromCurrent(int scatterChoice) {
        Pose shootTarget = getShootPoseForScatterChoice(scatterChoice);
        Pose current = getCurrentPoseOr(shootTarget);

        return follower.pathBuilder()
                .addPath(new BezierLine(current, shootTarget))
                .setLinearHeadingInterpolation(current.getHeading(), shootTarget.getHeading())
                .addParametricCallback(0.5, () -> autoManipulator.hold())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();
    }


    private boolean isAutoManipulatorShootingState() {
        switch (pathState) {
            case 2:
            case 4:
            case 6:
                return true;

            default:
                return false;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected Follower getFollower() {
        return follower;
    }

    private double headingFrom(Pose from, Pose to) {
        return Math.atan2(
                to.getY() - from.getY(),
                to.getX() - from.getX()
        );
    }

    private int getScatterChoiceForCycle(int cycleIndex) {
        if (scatterPlan == null || scatterPlan.length == 0) {
            return 1;
        }

        if (cycleIndex < 0 || cycleIndex >= scatterPlan.length) {
            return 1;
        }

        int choice = scatterPlan[cycleIndex];

        if (choice < 0 || choice > 2) {
            return 1;
        }

        return choice;
    }

    private PathChain getScatterPathToRun(int cycleIndex) {
        Pose shootStart = getCurrentPoseOr(getExpectedScatterStartShootPose(cycleIndex));

        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return buildScatterAToCollected(shootStart);
            case 2:
                return buildScatterCFull(shootStart);
            case 1:
            default:
                return buildScatterBToCollected(shootStart);
        }
    }

    private Pose getExpectedScatterStartShootPose(int cycleIndex) {
        if (cycleIndex <= 0) {
            return ShootAfterTripleC;
        }

        return getShootPoseForScatterChoice(getScatterChoiceForCycle(cycleIndex - 1));
    }

    private Pose getShootPoseForScatterChoice(int scatterChoice) {
        switch (scatterChoice) {
            case 0:
                return ShootScatterA;
            case 2:
                // Scatter C currently finishes through the B-side collected point,
                // so it uses the Scatter B shooting point instead of adding a fifth point.
            case 1:
            default:
                return ShootScatterB;
        }
    }

    private PathChain getScatterReturnToShoot(int cycleIndex) {
        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return scatterAReturnToShoot;
            case 1:
            default:
                return scatterBReturnToShoot;
        }
    }

    private boolean currentScatterNeedsCollectedWait() {
        int choice = getScatterChoiceForCycle(scatterCycleIndex);
        return choice == 0 || choice == 1;
    }

    private String scatterChoiceToString(int choice) {
        switch (choice) {
            case 0:
                return "A";
            case 2:
                return "C";
            case 1:
            default:
                return "B";
        }
    }

    private String getScatterPlanString() {
        if (scatterPlan == null || scatterPlan.length == 0) {
            return "";
        }

        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < SCATTER_CYCLE_COUNT; i++) {
            if (i > 0) {
                builder.append(" ");
            }

            builder.append(scatterChoiceToString(getScatterChoiceForCycle(i)));
        }

        return builder.toString();
    }
}