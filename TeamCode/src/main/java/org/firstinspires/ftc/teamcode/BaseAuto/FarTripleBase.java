package org.firstinspires.ftc.teamcode.BaseAuto;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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

    private double intakeAToCollectedTimeSec = Double.NaN;
    private double intakeBToCollectedTimeSec = Double.NaN;
    private double intakeCToCollectedTimeSec = Double.NaN;
    private double intakeAToCollectedMaxTimeSec = Double.NaN;
    private double intakeBToCollectedMaxTimeSec = Double.NaN;
    private double intakeCToCollectedMaxTimeSec = Double.NaN;
    private double currentIntakeSegmentElapsedSec = Double.NaN;
    private int intakeATimingSampleCount = 0;
    private int intakeBTimingSampleCount = 0;
    private int intakeCTimingSampleCount = 0;

    private double intakeSegmentStartSec = 0.0;
    private boolean timingIntakeSegment = false;
    private boolean intakeSegmentTimerStarted = false;
    private boolean watchIntakeSegmentTimer = false;
    private int watchedScatterChoice = -1;

    /**
     * Master telemetry toggle for this autonomous OpMode.
     *
     * true:
     * - enables every telemetry source in this class;
     * - enables subsystem telemetry;
     * - measures the complete Intake A/B/C -> Collected segment.
     *
     * false:
     * - suppresses all telemetry output.
     */
    public static boolean USE_TELEMETRY = false;

    /**
     * Set true only while deliberately calibrating complete intake-segment times.
     * During normal autonomous runs, keep this false so the robot cannot wait
     * forever when the follower reaches a collected point but remains busy.
     */
    public static boolean BYPASS_INTAKE_TIMEOUTS_FOR_CALIBRATION = false;

    private static final double INTAKE_TIMING_TIMEOUT_MARGIN_SEC = 0.05;
    private static final double INTAKE_A_TO_COLLECTED_TIMEOUT_SEC = 0.6;
    private static final double INTAKE_B_TO_COLLECTED_TIMEOUT_SEC = 0.6;
    // Start C with B's timeout behavior; telemetry can be used to calibrate it independently.
    private static final double INTAKE_C_TO_COLLECTED_TIMEOUT_SEC =
            INTAKE_B_TO_COLLECTED_TIMEOUT_SEC;

    private static final int SCATTER_CYCLE_COUNT = 6;
    private static final double COLLECTED_SCATTER_WAIT_SEC = 0.25;

    public static int[] scatterPlan = {0, 0, 1, 0, 1, 0};
    public static Pose finalPose;

    private static final double SHOOTER_VELOCITY = 1940;
    private static final double PRELOAD_SHOOT_DELAY = 0.0;

    /**
     * AUTO uses an axial heading only when the straight forward/backward hold is
     * long enough to be useful and the axial detour does not add too much rotation.
     *
     * FORCE_AXIAL and FORCE_DIRECT make it easy to A/B time the same autonomous
     * without changing any path coordinates.
     */
    public enum TransferHeadingPolicy {
        AUTO,
        FORCE_AXIAL,
        FORCE_DIRECT
    }

    public static TransferHeadingPolicy transferHeadingPolicy =
            TransferHeadingPolicy.AUTO;

    /*
     * Translation is fastest when the robot's forward axis is parallel to the path.
     * TURN_IN_INCHES_PER_RAD estimates how much line distance is required per radian
     * of heading change. Tune it from real robot data.
     */
    private static final double TURN_IN_BASE_INCHES = 2.0;
    private static final double TURN_IN_INCHES_PER_RAD = 8.0;

    /*
     * Physical minimums are used instead of fixed t-value minimums. Since every
     * path passed to the optimized heading builder is currently a BezierLine,
     * line-distance divided by path length maps directly to a t-window.
     */
    private static final double MIN_TURN_WINDOW_INCHES = 3.0;
    private static final double MIN_AXIAL_HOLD_INCHES = 3.0;

    private static final double HEADING_EPSILON_RAD =
            Math.toRadians(2.0);

    private static final double MIN_USEFUL_AXIAL_HOLD_INCHES = 8.0;
    private static final double MAX_AXIAL_EXTRA_TURN_RAD =
            Math.toRadians(15.0);

    private static final double PRELOAD_AIM_END_T = 0.69;
    private static final double INTAKE_ALIGN_END_T = 0.82;
    private static final double SHOOT_AIM_END_T = 0.79;


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

    private Pose p(double x, double y, double headingDeg) {
        return FieldMirror.pose(getAlliance(), x, y, headingDeg);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        addTelemetryData("Status", "Initializing...");

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

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Ready");
        addTelemetryData("Scatter Plan", getScatterPlanString());
        addTelemetryData("Heading Policy", transferHeadingPolicy);
        addIntakeTimingTelemetry();
        updateTelemetryOutput();
    }

    @Override
    public void init_loop() {
        Pose pose = follower.getPose();

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Waiting for Start");
        addTelemetryData("Scatter Plan", getScatterPlanString());
        addTelemetryData("Heading Policy", transferHeadingPolicy);

        if (pose != null) {
            addTelemetryData("Robot X", pose.getX());
            addTelemetryData("Robot Y", pose.getY());
            addTelemetryData("Robot Heading", Math.toDegrees(pose.getHeading()));
        }

        if (USE_TELEMETRY) {
            autoManipulator.addTelemetry();
        }
        addIntakeTimingTelemetry();
        updateTelemetryOutput();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        resetLoopTiming();

        scatterCycleIndex = 0;
        resetIntakeSegmentTimers();

        lastVoltageUpdateNs = 0L;
        wasVoltageFastModeLastLoop = false;

        setPathState(0);

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Started");
        addTelemetryData("Scatter Plan", getScatterPlanString());
        addTelemetryData("Heading Policy", transferHeadingPolicy);
        addIntakeTimingTelemetry();
        updateTelemetryOutput();
    }

    @Override
    public void loop() {
        prepareLoopTiming();

        follower.update();

        Pose pose = follower.getPose();
        if (pose == null) {
            addTelemetryData("Robot Pose", "Unavailable");
            updateTelemetryOutput();
            return;
        }

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
        addIntakeTimingTelemetry();
        updateTelemetryOutput();
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

        addTelemetryData("Status", "Stopped");
        updateTelemetryOutput();
    }

    private void addTelemetryData(String caption, Object value) {
        if (USE_TELEMETRY && telemetry != null) {
            telemetry.addData(caption, value);
        }
    }

    private void addTelemetryData(String caption, String format, Object... args) {
        if (USE_TELEMETRY && telemetry != null) {
            telemetry.addData(caption, format, args);
        }
    }

    /**
     * Sends the current telemetry frame only when enabled. When disabled, it
     * clears anything a subsystem may have added directly to the shared telemetry
     * object and returns without publishing a telemetry frame.
     */
    private void updateTelemetryOutput() {
        if (telemetry == null) {
            return;
        }

        if (!USE_TELEMETRY) {
            // Remove anything added directly by Intake, Shooter, or
            // AutoManipulator before the SDK can publish the frame.
            telemetry.clearAll();
            return;
        }

        telemetry.update();
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

        IntakeScatterA = p(27, 12, 180);
        CollectedScatterA = p(9, 6, 180);

        IntakeC = p(44, 35, 180);
        CollectedC = p(19, 35, 180);

        IntakeScatterB = p(27, 21, 180);
        CollectedScatterB = p(12, 21, 180);

        IntakeScatterC = p(27, 35, 180);
        CollectedScatterC = p(12, 35, 180);

        ShootPreloadPoint = p(55.1, 13.5, -70);
        ShootAfterTripleC = p(55.1, 13.5, -75.5);
        ShootScatterA = p(55.1, 13.5, -76);
        ShootScatterB = p(55.1, 13.5, -73.6);
    }

    private void buildPaths() {
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPreloadPoint))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                startPose,
                                ShootPreloadPoint,
                                ShootPreloadPoint.getHeading(),
                                PRELOAD_AIM_END_T
                        )
                )
                .addParametricCallback(0.7, () -> autoManipulator.releaseForShot())
                .build();

        ThirdFull = follower.pathBuilder()
                .addPath(new BezierLine(ShootPreloadPoint, IntakeC))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                ShootPreloadPoint,
                                IntakeC,
                                IntakeC.getHeading(),
                                INTAKE_ALIGN_END_T
                        )
                )

                /*
                 * These collection strokes are deliberately pure forward axial
                 * motion. Do not replace them with the adaptive transfer profile.
                 */
                .addPath(new BezierLine(IntakeC, CollectedC))
                .setConstantHeadingInterpolation(IntakeC.getHeading())

                .addPath(new BezierLine(CollectedC, ShootAfterTripleC))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                CollectedC,
                                ShootAfterTripleC,
                                ShootAfterTripleC.getHeading(),
                                SHOOT_AIM_END_T
                        )
                )
                .addParametricCallback(0.5, () -> autoManipulator.hold())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();
    }

    private PathChain buildScatterAToCollected(Pose shootStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterA))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                shootStart,
                                IntakeScatterA,
                                IntakeScatterA.getHeading(),
                                INTAKE_ALIGN_END_T
                        )
                )

                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())
                .build();
    }

    private PathChain buildScatterBToCollected(Pose shootStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterB))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                shootStart,
                                IntakeScatterB,
                                IntakeScatterB.getHeading(),
                                INTAKE_ALIGN_END_T
                        )
                )

                .addPath(new BezierLine(IntakeScatterB, CollectedScatterB))
                .setConstantHeadingInterpolation(IntakeScatterB.getHeading())
                .build();
    }

    private PathChain buildScatterCToCollected(Pose shootStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootStart, IntakeScatterC))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                shootStart,
                                IntakeScatterC,
                                IntakeScatterC.getHeading(),
                                INTAKE_ALIGN_END_T
                        )
                )

                .addPath(new BezierLine(IntakeScatterC, CollectedScatterC))
                .setConstantHeadingInterpolation(IntakeScatterC.getHeading())
                .build();
    }

    private PathChain buildReturnToShoot(Pose start, Pose shootTarget) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, shootTarget))
                .setHeadingInterpolation(
                        buildOptimizedStraightHeading(
                                start,
                                shootTarget,
                                shootTarget.getHeading(),
                                SHOOT_AIM_END_T
                        )
                )
                .addParametricCallback(0.5, () -> autoManipulator.hold())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();
    }

    /**
     * Selects between two continuous heading profiles for a straight line:
     *
     * 1. Axial: align forward/backward with the line, hold that heading, then
     *    rotate to the required endpoint heading.
     * 2. Direct: make only the shortest start-heading-to-end-heading turn.
     *
     * AUTO uses the axial profile only when it creates a useful amount of pure
     * axial travel without paying an excessive extra-rotation penalty.
     */
    private HeadingInterpolator buildOptimizedStraightHeading(
            Pose start,
            Pose end,
            double finalHeading,
            double requestedFinalTurnEndT
    ) {
        final boolean backwards = shouldDriveBackward(
                start,
                end,
                finalHeading
        );
        final double axialHeading =
                getAxialTravelHeading(start, end, backwards);

        final TransferHeadingPolicy activePolicy =
                transferHeadingPolicy != null
                        ? transferHeadingPolicy
                        : TransferHeadingPolicy.AUTO;

        switch (activePolicy) {
            case FORCE_AXIAL:
                return buildContinuousAxialHeading(
                        start,
                        end,
                        axialHeading,
                        finalHeading,
                        requestedFinalTurnEndT
                );

            case FORCE_DIRECT:
                return buildDirectShortestTurnHeading(
                        start,
                        end,
                        finalHeading,
                        requestedFinalTurnEndT
                );

            case AUTO:
            default:
                if (shouldUseAxialHeading(
                        start,
                        end,
                        axialHeading,
                        finalHeading,
                        requestedFinalTurnEndT
                )) {
                    return buildContinuousAxialHeading(
                            start,
                            end,
                            axialHeading,
                            finalHeading,
                            requestedFinalTurnEndT
                    );
                }

                return buildDirectShortestTurnHeading(
                        start,
                        end,
                        finalHeading,
                        requestedFinalTurnEndT
                );
        }
    }

    private boolean shouldUseAxialHeading(
            Pose start,
            Pose end,
            double axialHeading,
            double finalHeading,
            double requestedFinalTurnEndT
    ) {
        final double usefulAxialHoldInches = getAxialHoldDistanceInches(
                start,
                end,
                axialHeading,
                finalHeading,
                requestedFinalTurnEndT
        );

        final double axialTurnRadians =
                angleDistance(start.getHeading(), axialHeading)
                        + angleDistance(axialHeading, finalHeading);

        final double directTurnRadians =
                angleDistance(start.getHeading(), finalHeading);

        final double extraTurnRadians =
                Math.max(0.0, axialTurnRadians - directTurnRadians);

        return usefulAxialHoldInches >= MIN_USEFUL_AXIAL_HOLD_INCHES
                && extraTurnRadians <= MAX_AXIAL_EXTRA_TURN_RAD;
    }

    /**
     * Mirrors the axial profile's geometry calculation without constructing the
     * interpolator, so AUTO can determine how much pure axial line distance the
     * candidate would actually provide.
     */
    private double getAxialHoldDistanceInches(
            Pose start,
            Pose end,
            double axialHeading,
            double finalHeading,
            double requestedFinalTurnEndT
    ) {
        final double pathLength = getPathLength(start, end);

        if (pathLength < 1e-6) {
            return 0.0;
        }

        final double minimumTurnWindowT =
                getMinimumTurnWindowT(pathLength, 1.0);

        final double finalTurnEndT = clamp(
                requestedFinalTurnEndT,
                minimumTurnWindowT,
                1.0
        );

        final double finalTurnStartT = getTurnStartT(
                start,
                end,
                axialHeading,
                finalHeading,
                finalTurnEndT
        );

        if (finalTurnStartT <= 1e-6) {
            return 0.0;
        }

        final double startTurnRadians =
                angleDistance(start.getHeading(), axialHeading);

        if (startTurnRadians <= HEADING_EPSILON_RAD) {
            return finalTurnStartT * pathLength;
        }

        final double requestedStartTurnDistance =
                TURN_IN_BASE_INCHES
                        + TURN_IN_INCHES_PER_RAD * startTurnRadians;

        final double minimumStartTurnWindowT =
                getMinimumTurnWindowT(pathLength, finalTurnStartT);

        double startAlignEndT = clamp(
                requestedStartTurnDistance / pathLength,
                minimumStartTurnWindowT,
                finalTurnStartT
        );

        final double axialHoldInches =
                Math.max(0.0, finalTurnStartT - startAlignEndT) * pathLength;

        if (axialHoldInches < MIN_AXIAL_HOLD_INCHES) {
            startAlignEndT = finalTurnStartT;
        }

        return Math.max(0.0, finalTurnStartT - startAlignEndT) * pathLength;
    }

    /**
     * Uses only the shortest required rotation between the actual start heading
     * and the endpoint heading. It holds the starting heading first when the line
     * is long enough, turns over a distance scaled to the required angle, and
     * holds the endpoint heading for final settling.
     */
    private HeadingInterpolator buildDirectShortestTurnHeading(
            Pose start,
            Pose end,
            double finalHeading,
            double requestedFinalTurnEndT
    ) {
        final double pathLength = getPathLength(start, end);

        if (pathLength < 1e-6) {
            return HeadingInterpolator.constant(finalHeading);
        }

        final double directTurnRadians =
                angleDistance(start.getHeading(), finalHeading);

        if (directTurnRadians <= HEADING_EPSILON_RAD) {
            return HeadingInterpolator.constant(finalHeading);
        }

        final double minimumTurnWindowT =
                getMinimumTurnWindowT(pathLength, 1.0);

        final double finalTurnEndT = clamp(
                requestedFinalTurnEndT,
                minimumTurnWindowT,
                1.0
        );

        final double requestedTurnDistance =
                TURN_IN_BASE_INCHES
                        + TURN_IN_INCHES_PER_RAD * directTurnRadians;

        final double requestedTurnWindowT = requestedTurnDistance / pathLength;
        final double availableMinimumTurnWindowT =
                getMinimumTurnWindowT(pathLength, finalTurnEndT);

        final double turnWindowT = clamp(
                requestedTurnWindowT,
                availableMinimumTurnWindowT,
                finalTurnEndT
        );

        final double turnStartT = clamp(
                finalTurnEndT - turnWindowT,
                0.0,
                finalTurnEndT - availableMinimumTurnWindowT
        );

        if (turnStartT <= 1e-6) {
            return HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode.linear(
                            0.0,
                            finalTurnEndT,
                            start.getHeading(),
                            finalHeading
                    ),
                    new HeadingInterpolator.PiecewiseNode(
                            finalTurnEndT,
                            1.0,
                            HeadingInterpolator.constant(finalHeading)
                    )
            );
        }

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0.0,
                        turnStartT,
                        HeadingInterpolator.constant(start.getHeading())
                ),
                HeadingInterpolator.PiecewiseNode.linear(
                        turnStartT,
                        finalTurnEndT,
                        start.getHeading(),
                        finalHeading
                ),
                new HeadingInterpolator.PiecewiseNode(
                        finalTurnEndT,
                        1.0,
                        HeadingInterpolator.constant(finalHeading)
                )
        );
    }

    /**
     * Builds a continuous axial heading profile for a straight travel segment:
     *
     * 1. Smoothly align from the actual starting heading to the selected
     *    forward/backward axial heading.
     * 2. Hold the axial heading for as much of the line as possible.
     * 3. Smoothly turn from the axial heading to the endpoint heading.
     * 4. Hold the endpoint heading for final settling.
     */
    private HeadingInterpolator buildContinuousAxialHeading(
            Pose start,
            Pose end,
            double axialHeading,
            double finalHeading,
            double requestedFinalTurnEndT
    ) {
        final double pathLength = getPathLength(start, end);

        if (pathLength < 1e-6) {
            return HeadingInterpolator.constant(finalHeading);
        }

        final double minimumTurnWindowT =
                getMinimumTurnWindowT(pathLength, 1.0);

        final double finalTurnEndT = clamp(
                requestedFinalTurnEndT,
                minimumTurnWindowT,
                1.0
        );

        final double finalTurnStartT = getTurnStartT(
                start,
                end,
                axialHeading,
                finalHeading,
                finalTurnEndT
        );

        if (finalTurnStartT <= 1e-6) {
            return HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode.linear(
                            0.0,
                            finalTurnEndT,
                            start.getHeading(),
                            finalHeading
                    ),
                    new HeadingInterpolator.PiecewiseNode(
                            finalTurnEndT,
                            1.0,
                            HeadingInterpolator.constant(finalHeading)
                    )
            );
        }

        final double startTurnRadians =
                angleDistance(start.getHeading(), axialHeading);

        if (startTurnRadians <= HEADING_EPSILON_RAD) {
            return HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0.0,
                            finalTurnStartT,
                            HeadingInterpolator.constant(axialHeading)
                    ),
                    HeadingInterpolator.PiecewiseNode.linear(
                            finalTurnStartT,
                            finalTurnEndT,
                            axialHeading,
                            finalHeading
                    ),
                    new HeadingInterpolator.PiecewiseNode(
                            finalTurnEndT,
                            1.0,
                            HeadingInterpolator.constant(finalHeading)
                    )
            );
        }

        final double requestedStartTurnDistance =
                TURN_IN_BASE_INCHES
                        + TURN_IN_INCHES_PER_RAD * startTurnRadians;

        final double minimumStartTurnWindowT =
                getMinimumTurnWindowT(pathLength, finalTurnStartT);

        double startAlignEndT = clamp(
                requestedStartTurnDistance / pathLength,
                minimumStartTurnWindowT,
                finalTurnStartT
        );

        final double axialHoldInches =
                Math.max(0.0, finalTurnStartT - startAlignEndT) * pathLength;

        if (axialHoldInches < MIN_AXIAL_HOLD_INCHES) {
            startAlignEndT = finalTurnStartT;
        }

        if (startAlignEndT >= finalTurnStartT - 1e-6) {
            return HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode.linear(
                            0.0,
                            finalTurnStartT,
                            start.getHeading(),
                            axialHeading
                    ),
                    HeadingInterpolator.PiecewiseNode.linear(
                            finalTurnStartT,
                            finalTurnEndT,
                            axialHeading,
                            finalHeading
                    ),
                    new HeadingInterpolator.PiecewiseNode(
                            finalTurnEndT,
                            1.0,
                            HeadingInterpolator.constant(finalHeading)
                    )
            );
        }

        return HeadingInterpolator.piecewise(
                HeadingInterpolator.PiecewiseNode.linear(
                        0.0,
                        startAlignEndT,
                        start.getHeading(),
                        axialHeading
                ),
                new HeadingInterpolator.PiecewiseNode(
                        startAlignEndT,
                        finalTurnStartT,
                        HeadingInterpolator.constant(axialHeading)
                ),
                HeadingInterpolator.PiecewiseNode.linear(
                        finalTurnStartT,
                        finalTurnEndT,
                        axialHeading,
                        finalHeading
                ),
                new HeadingInterpolator.PiecewiseNode(
                        finalTurnEndT,
                        1.0,
                        HeadingInterpolator.constant(finalHeading)
                )
        );
    }

    private double getPathLength(Pose start, Pose end) {
        return Math.hypot(
                end.getX() - start.getX(),
                end.getY() - start.getY()
        );
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
                if (!follower.isBusy()) {
                    if (currentScatterNeedsCollectedWait()) {
                        setPathState(55);
                    } else {
                        autoManipulator.shoot();
                        setPathState(6);
                    }
                }
                break;

            case 55:
                if (pathTimer.getElapsedTimeSeconds() >= COLLECTED_SCATTER_WAIT_SEC) {
                    cancelIntakeSegmentTimer();
                    autoManipulator.hold();
                    follower.followPath(
                            returnToShootFromCurrent(
                                    getScatterChoiceForCycle(scatterCycleIndex)
                            ),
                            1.0,
                            true
                    );
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
        intakeCToCollectedTimeSec = Double.NaN;
        intakeAToCollectedMaxTimeSec = Double.NaN;
        intakeBToCollectedMaxTimeSec = Double.NaN;
        intakeCToCollectedMaxTimeSec = Double.NaN;
        currentIntakeSegmentElapsedSec = Double.NaN;
        intakeATimingSampleCount = 0;
        intakeBTimingSampleCount = 0;
        intakeCTimingSampleCount = 0;
        cancelIntakeSegmentTimer();
    }

    private void armIntakeSegmentTimerForCycle() {
        watchedScatterChoice = getScatterChoiceForCycle(scatterCycleIndex);
        watchIntakeSegmentTimer = watchedScatterChoice == 0
                || watchedScatterChoice == 1
                || watchedScatterChoice == 2;
        timingIntakeSegment = false;
        intakeSegmentTimerStarted = false;
        intakeSegmentStartSec = 0.0;
        currentIntakeSegmentElapsedSec = Double.NaN;
    }

    private void cancelIntakeSegmentTimer() {
        watchIntakeSegmentTimer = false;
        timingIntakeSegment = false;
        intakeSegmentTimerStarted = false;
        intakeSegmentStartSec = 0.0;
        currentIntakeSegmentElapsedSec = Double.NaN;
        watchedScatterChoice = -1;
    }

    private void updateIntakeSegmentTiming() {
        if (!watchIntakeSegmentTimer || pathState != 5) {
            return;
        }

        final double nowSec = opmodeTimer.getElapsedTimeSeconds();
        final boolean followerBusy = follower.isBusy();
        final boolean onIntakeToCollectedSegment =
                followerBusy && follower.getCurrentPathNumber() >= 1;

        if (!timingIntakeSegment && !intakeSegmentTimerStarted
                && onIntakeToCollectedSegment) {
            timingIntakeSegment = true;
            intakeSegmentTimerStarted = true;
            intakeSegmentStartSec = nowSec;
            currentIntakeSegmentElapsedSec = 0.0;
        }

        if (!timingIntakeSegment) {
            return;
        }

        final double elapsedSec = nowSec - intakeSegmentStartSec;
        currentIntakeSegmentElapsedSec = elapsedSec;

        // The path chain has finished, so this is the complete Intake -> Collected time.
        if (!followerBusy) {
            recordCompletedIntakeSegmentTime(watchedScatterChoice, elapsedSec);
            timingIntakeSegment = false;
            return;
        }

        // During normal autonomous operation, leave the collected point when the
        // configured segment timeout expires, even if Pedro still reports busy.
        // Bypass this only while deliberately calibrating complete segment times.
        if (!BYPASS_INTAKE_TIMEOUTS_FOR_CALIBRATION
                && elapsedSec >= getIntakeSegmentTimeoutSec(watchedScatterChoice)) {
            timeoutBackToShoot();
        }
    }

    private void recordCompletedIntakeSegmentTime(int scatterChoice, double elapsedSec) {
        if (scatterChoice == 0) {
            intakeAToCollectedTimeSec = elapsedSec;
            intakeAToCollectedMaxTimeSec = maxIgnoringNaN(
                    intakeAToCollectedMaxTimeSec,
                    elapsedSec
            );
            intakeATimingSampleCount++;
        } else if (scatterChoice == 1) {
            intakeBToCollectedTimeSec = elapsedSec;
            intakeBToCollectedMaxTimeSec = maxIgnoringNaN(
                    intakeBToCollectedMaxTimeSec,
                    elapsedSec
            );
            intakeBTimingSampleCount++;
        } else if (scatterChoice == 2) {
            intakeCToCollectedTimeSec = elapsedSec;
            intakeCToCollectedMaxTimeSec = maxIgnoringNaN(
                    intakeCToCollectedMaxTimeSec,
                    elapsedSec
            );
            intakeCTimingSampleCount++;
        }
    }

    private double maxIgnoringNaN(double currentMax, double value) {
        return Double.isNaN(currentMax) ? value : Math.max(currentMax, value);
    }

    private double getIntakeSegmentTimeoutSec(int scatterChoice) {
        switch (scatterChoice) {
            case 0:
                return INTAKE_A_TO_COLLECTED_TIMEOUT_SEC;
            case 2:
                return INTAKE_C_TO_COLLECTED_TIMEOUT_SEC;
            case 1:
            default:
                return INTAKE_B_TO_COLLECTED_TIMEOUT_SEC;
        }
    }

    private String formatTimeSec(double timeSec) {
        if (Double.isNaN(timeSec)) {
            return "--";
        }
        return String.format(Locale.US, "%.3f sec", timeSec);
    }

    private String formatSuggestedTimeoutSec(double maxTimeSec) {
        if (Double.isNaN(maxTimeSec)) {
            return "--";
        }
        return String.format(
                Locale.US,
                "%.3f sec",
                maxTimeSec + INTAKE_TIMING_TIMEOUT_MARGIN_SEC
        );
    }

    private void addIntakeTimingTelemetry() {
        if (!USE_TELEMETRY) {
            return;
        }

        addTelemetryData(
                "Intake timeout mode",
                BYPASS_INTAKE_TIMEOUTS_FOR_CALIBRATION
                        ? "BYPASSED (calibration only)"
                        : "ENFORCED"
        );

        if (timingIntakeSegment && watchedScatterChoice >= 0) {
            addTelemetryData(
                    "Timing now",
                    "%s: %s",
                    scatterChoiceToString(watchedScatterChoice),
                    formatTimeSec(currentIntakeSegmentElapsedSec)
            );
        } else {
            addTelemetryData("Timing now", "--");
        }

        addTelemetryData(
                "A last / max / samples",
                "%s / %s / %d",
                formatTimeSec(intakeAToCollectedTimeSec),
                formatTimeSec(intakeAToCollectedMaxTimeSec),
                intakeATimingSampleCount
        );
        addTelemetryData(
                "A suggested timeout",
                formatSuggestedTimeoutSec(intakeAToCollectedMaxTimeSec)
        );

        addTelemetryData(
                "B last / max / samples",
                "%s / %s / %d",
                formatTimeSec(intakeBToCollectedTimeSec),
                formatTimeSec(intakeBToCollectedMaxTimeSec),
                intakeBTimingSampleCount
        );
        addTelemetryData(
                "B suggested timeout",
                formatSuggestedTimeoutSec(intakeBToCollectedMaxTimeSec)
        );

        addTelemetryData(
                "C last / max / samples",
                "%s / %s / %d",
                formatTimeSec(intakeCToCollectedTimeSec),
                formatTimeSec(intakeCToCollectedMaxTimeSec),
                intakeCTimingSampleCount
        );
        addTelemetryData(
                "C suggested timeout",
                formatSuggestedTimeoutSec(intakeCToCollectedMaxTimeSec)
        );
    }

    private void startNextScatterOrLeave() {

        if (scatterCycleIndex < SCATTER_CYCLE_COUNT) {
            autoManipulator.intake();
            armIntakeSegmentTimerForCycle();
            follower.followPath(getScatterPathToRun(scatterCycleIndex), 1.0, true);
            setPathState(5);
        } else {
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
        autoManipulator.hold();
        follower.followPath(returnToShootFromCurrent(scatterChoice), 1.0, true);
        setPathState(56);
    }

    private PathChain returnToShootFromCurrent(int scatterChoice) {
        Pose shootTarget = getShootPoseForScatterChoice(scatterChoice);
        Pose collectedFallback = getCollectedPoseForScatterChoice(scatterChoice);
        Pose current = getCurrentPoseOr(collectedFallback);
        return buildReturnToShoot(current, shootTarget);
    }

    private boolean shouldDriveBackward(
            Pose start,
            Pose end,
            double finalHeading
    ) {
        final double forwardHeading =
                getAxialTravelHeading(start, end, false);

        final double backwardHeading =
                getAxialTravelHeading(start, end, true);

        final double forwardTurnCost =
                angleDistance(start.getHeading(), forwardHeading)
                        + angleDistance(forwardHeading, finalHeading);

        final double backwardTurnCost =
                angleDistance(start.getHeading(), backwardHeading)
                        + angleDistance(backwardHeading, finalHeading);

        return backwardTurnCost < forwardTurnCost;
    }

    private double getAxialTravelHeading(Pose start, Pose end, boolean backwards) {
        final double dx = end.getX() - start.getX();
        final double dy = end.getY() - start.getY();

        if (Math.hypot(dx, dy) < 1e-6) {
            return normalizeAngle(end.getHeading());
        }

        final double pathHeading = Math.atan2(dy, dx);
        return normalizeAngle(pathHeading + (backwards ? Math.PI : 0.0));
    }

    private double getTurnStartT(
            Pose start,
            Pose end,
            double axialHeading,
            double finalHeading,
            double turnEndT
    ) {
        final double pathLength = getPathLength(start, end);

        if (pathLength < 1e-6 || turnEndT <= 1e-6) {
            return 0.0;
        }

        final double turnRadians = angleDistance(axialHeading, finalHeading);
        final double requestedTurnDistance =
                TURN_IN_BASE_INCHES + TURN_IN_INCHES_PER_RAD * turnRadians;

        final double requestedWindowT = requestedTurnDistance / pathLength;
        final double minimumTurnWindowT =
                getMinimumTurnWindowT(pathLength, turnEndT);

        final double turnWindowT = clamp(
                requestedWindowT,
                minimumTurnWindowT,
                turnEndT
        );

        return clamp(
                turnEndT - turnWindowT,
                0.0,
                turnEndT - minimumTurnWindowT
        );
    }

    /**
     * Converts the physical minimum turn distance to a t-window for a straight
     * BezierLine. The maximum keeps the returned minimum inside the available
     * section of the path.
     */
    private double getMinimumTurnWindowT(
            double pathLength,
            double maximumWindowT
    ) {
        if (pathLength < 1e-6 || maximumWindowT <= 0.0) {
            return 0.0;
        }

        return Math.min(
                maximumWindowT,
                MIN_TURN_WINDOW_INCHES / pathLength
        );
    }

    private double angleDistance(double a, double b) {
        return Math.abs(normalizeAngle(b - a));
    }

    private double normalizeAngle(double angle) {
        double normalized = angle % (2.0 * Math.PI);

        if (normalized >= Math.PI) {
            normalized -= 2.0 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2.0 * Math.PI;
        }

        return normalized;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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

    private int getScatterChoiceForCycle(int cycleIndex) {
        if (scatterPlan == null || scatterPlan.length == 0) {
            return 1;
        }

        if (cycleIndex < 0 || cycleIndex >= scatterPlan.length) {
            return 1;
        }

        int choice = scatterPlan[cycleIndex];

        // Valid scatter choices are 0 (A), 1 (B), and 2 (C).
        if (choice != 0 && choice != 1 && choice != 2) {
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
                return buildScatterCToCollected(shootStart);
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
                return ShootAfterTripleC;
            case 1:
            default:
                return ShootScatterB;
        }
    }

    private Pose getCollectedPoseForScatterChoice(int scatterChoice) {
        switch (scatterChoice) {
            case 0:
                return CollectedScatterA;
            case 2:
                return CollectedScatterC;
            case 1:
            default:
                return CollectedScatterB;
        }
    }

    private boolean currentScatterNeedsCollectedWait() {
        int choice = getScatterChoiceForCycle(scatterCycleIndex);
        return choice == 0 || choice == 1 || choice == 2;
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