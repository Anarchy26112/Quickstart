package org.firstinspires.ftc.teamcode.BaseAuto;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private Limelight3A limelight;
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
    private int currentScatterChoice = 1; // 0=A, 1=B, 2=C

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
    public static boolean USE_TELEMETRY = true;

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

    /*
     * Optimized scatter geometry.
     *
     * The collected endpoints remain fixed. Each collection entry point is
     * placed exactly this distance before its endpoint, which is the shortest
     * possible route for the selected collection-stroke requirement.
     *
     * The approach is a cubic Bezier whose endpoint tangent matches the
     * horizontal collection stroke, eliminating the old sharp line-to-line
     * corner at IntakeScatterA/B/C.
     */
    private static final double SCATTER_COLLECTED_X = 11.15;
    private static final double SCATTER_COLLECTION_RUN_INCHES = 10.0;
    private static final double SCATTER_CURVE_HANDLE_RATIO = 1.0 / 3.0;
    private static final double SCATTER_CURVE_MIN_HANDLE_INCHES = 4.0;
    private static final double SCATTER_CURVE_MAX_HANDLE_INCHES = 12.0;

    // Limelight setup. Change these two values only if your configuration differs.
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int LIMELIGHT_PIPELINE = 0;

    // The Python pipeline uses llpython[0] as valid and llpython[1] as path 1/2/3.
    // A 250 ms pulse is long enough for the Limelight Python pipeline to observe
    // at least one high llrobot[0] frame even when the camera or NT update is late.
    private static final double VISION_RESET_PULSE_SEC = 0.15;;

    /*
     * After the reset pulse goes low, allow enough time for the Python pipeline
     * to collect REQUIRED_STABLE_FRAMES consecutive winners. The wider timeout
     * and staleness allowance prevent a valid SnapScript result from being
     * rejected just because one Control Hub or camera frame arrived late.
     */
    private static final double VISION_READ_DELAY_SEC = 0.35;
    private static final double VISION_TIMEOUT_SEC = 1.20;
    private static final long MAX_VISION_STALENESS_MS = 500;
    private static final int DEFAULT_SCATTER_CHOICE = 1; // B

    /**
     * Diagnostic escape hatch for the reset-timing hypothesis.
     *
     * Set true to skip the reset pulse entirely. If vision reads start succeeding
     * with this on and fail with it off, the pulse (or the recovery time after it)
     * is confirmed as the cause.
     *
     * This is NOT safe for a real match: without the reset, a stale decision from
     * the previous scatter cycle can be accepted as if it were current.
     */
    public static boolean DISABLE_VISION_RESET_FOR_TESTING = false;

    /**
     * Controls the heading commanded for the Limelight scan.
     *
     * false: the original hardcoded 180 degrees, which was visually confirmed to
     *        aim correctly on the alliance tested so far.
     * true:  VisionScanPoint.getHeading(), which routes through FieldMirror like
     *        every other pose in this file.
     *
     * These differ only if FieldMirror mirrors heading. Check the
     * "Scan heading hardcoded / mirrored" telemetry on BOTH alliances before
     * changing this. If the two numbers are identical on red and blue, the
     * settings are equivalent and this flag can be ignored.
     */
    public static boolean MIRROR_SCAN_HEADING = false;

    // -------------------------------------------------------------------------
    // Vision diagnostics
    // -------------------------------------------------------------------------
    /*
     * The Python pipeline packs eight values into llpython:
     *
     *   [0] decision_valid   0 or 1
     *   [1] path number      1..3 (DEFAULT_PATH when nothing was decided)
     *   [2] current-frame raw score, path 1
     *   [3] current-frame raw score, path 2
     *   [4] current-frame raw score, path 3
     *   [5] Python reset sequence (increments on each reset rising edge)
     *   [6] Python frame heartbeat (increments every processed image)
     *   [7] measured pipeline FPS
     *
     * "valid = 0, path = 1" on its own is ambiguous: it is produced by the
     * no-target branch, by the not-yet-confirmed branch, AND by the pipeline's
     * top-level exception handler. Indices 2-4 and 7 are what separate them, so
     * everything the pipeline returns is latched and printed below.
     */
    private double[] lastPythonOutput = null;
    private String lastVisionReadStatus = "--";
    private String lastVisionBranch = "--";
    private long lastVisionStalenessMs = -1L;
    private double maxPathScoreSeen = 0.0;
    private double maxPathScoreThisCycle = 0.0;
    private int visionReadAttempts = 0;
    private int visionNoResultCount = 0;
    private int visionStaleCount = 0;
    private int visionInvalidFlagCount = 0;
    private int visionAcceptedCount = 0;
    private double commandedScanHeadingRad = Double.NaN;

    // Python increments llpython[5] whenever it observes a rising reset pulse.
    // Java requires that new reset sequence before accepting the next decision.
    private long expectedVisionResetSequence = -1L;
    private long observedVisionResetSequence = -1L;
    private long scanStartHeartbeat = -1L;
    private int visionResetNotObservedCount = 0;
    private int visionOldFrameCount = 0;

    // Python places a monotonically increasing frame heartbeat in llpython[6].
    // This proves whether the Control Hub is receiving new SnapScript frames.
    private long lastPythonFrameHeartbeat = -1L;
    private int repeatedPythonHeartbeatCount = 0;

    // One short result string per scatter cycle, e.g. "P2" or "TO->B".
    private final String[] visionCycleResults = new String[SCATTER_CYCLE_COUNT];

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

    // Explicit vision scan pose: x, y, and heading are defined together.
    private Pose VisionScanPoint;


    private PathChain ShootPreload;
    private PathChain ThirdFull;

    private Pose p(double x, double y, double headingDeg) {
        return FieldMirror.pose(getAlliance(), x, y, headingDeg);
    }

    /** Select any pipeline slot different from the SnapScript slot for recovery. */
    private int getLimelightRecoveryPipeline() {
        return LIMELIGHT_PIPELINE == 0 ? 1 : 0;
    }

    /** Small blocking delays are used only during init to avoid SDK issue #1895. */
    private void sleepForLimelight(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        addTelemetryData("Status", "Initializing...");

        initializeHubs();

        follower = Constants.createFollower(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);

        /*
         * IMPORTANT: Do not repeatedly call pipelineSwitch() during the OpMode.
         * FTC SDK issue #1895 documents that rapid/subsequent pipeline switches
         * can leave getLatestResult() timestamps updating while SnapScript's
         * llpython array remains frozen. Recover once here by switching away,
         * waiting, and switching back. After this method, the pipeline is never
         * switched again during the autonomous run.
         */
        limelight.start();
        sleepForLimelight(100);
        limelight.pipelineSwitch(getLimelightRecoveryPipeline());
        sleepForLimelight(100);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        sleepForLimelight(150);
        limelight.updatePythonInputs(new double[8]);

        buildPoses();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        autoManipulator = new AutoManipulator(intake, gate, telemetry);

        autoManipulator.setIntakePower(1.0, 1.0);
        autoManipulator.setHoldingPower(0.0, 0.0);
        autoManipulator.setShootingFeedPower(1.0, 1.0);

        buildPaths();

        resetVisionDiagnostics();

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Ready");
        addTelemetryData("Vision Scatter", scatterChoiceToString(currentScatterChoice));
        addTelemetryData("Heading Policy", transferHeadingPolicy);
        addIntakeTimingTelemetry();
        addVisionDiagnosticTelemetry();
        updateTelemetryOutput();
    }

    @Override
    public void init_loop() {
        // Some subsystems/OpModes disable telemetry auto-clear. In that case,
        // repeatedly calling addData() creates old duplicate rows that appear
        // frozen on the Driver Station. Force one clean telemetry frame here.
        if (USE_TELEMETRY && telemetry != null) {
            telemetry.clearAll();
        }

        Pose pose = follower.getPose();

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Waiting for Start");
        addTelemetryData("Vision Scatter", scatterChoiceToString(currentScatterChoice));
        addTelemetryData("Heading Policy", transferHeadingPolicy);
        addTelemetryData("LL running / connected", "%s / %s",
                limelight.isRunning(), limelight.isConnected());

        if (limelight != null && !limelight.isRunning()) {
            limelight.start();
        }

        if (pose != null) {
            addTelemetryData("Robot X", pose.getX());
            addTelemetryData("Robot Y", pose.getY());
            addTelemetryData("Robot Heading", Math.toDegrees(pose.getHeading()));
        }

        // Show both candidate scan headings so the mirrored-vs-hardcoded question
        // can be settled from the driver station on each alliance, before start.
        addTelemetryData(
                "Scan heading hardcoded / mirrored (deg)",
                "%.1f / %.1f",
                180.0,
                Math.toDegrees(VisionScanPoint.getHeading())
        );

        // Reading the pipeline before start is safe and shows whether the
        // Limelight is producing usable output at all while the robot sits still.
        // Note: init never sends a reset pulse, so this is the clean baseline.
        peekVisionPipeline("Init peek");

        if (USE_TELEMETRY) {
            autoManipulator.addTelemetry();
        }
        addIntakeTimingTelemetry();
        addVisionDiagnosticTelemetry();
        updateTelemetryOutput();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        resetLoopTiming();

        scatterCycleIndex = 0;
        currentScatterChoice = DEFAULT_SCATTER_CHOICE;
        resetIntakeSegmentTimers();
        resetVisionDiagnostics();
        expectedVisionResetSequence = -1L;
        observedVisionResetSequence = -1L;
        scanStartHeartbeat = -1L;
        limelight.updatePythonInputs(new double[8]);

        lastVoltageUpdateNs = 0L;
        wasVoltageFastModeLastLoop = false;

        setPathState(0);

        addTelemetryData("Alliance", getAlliance());
        addTelemetryData("Status", "Started");
        addTelemetryData("Vision Scatter", scatterChoiceToString(currentScatterChoice));
        addTelemetryData("Heading Policy", transferHeadingPolicy);
        addIntakeTimingTelemetry();
        addVisionDiagnosticTelemetry();
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

        addTelemetryData("LL running / connected", "%s / %s",
                limelight.isRunning(), limelight.isConnected());

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

        // Keep the displayed P1/P2/P3 values live throughout autonomous.
        // This is observation only; path selection still happens exclusively
        // inside state 8 and must pass the current scan-ID checks.
        peekVisionPipeline("live auto");

        updateIntakeSegmentTiming();
        autonomousPathUpdate();

        addTelemetryData("Path State", pathState);
        addTelemetryData("Robot X", "%.2f", pose.getX());
        addTelemetryData("Robot Y", "%.2f", pose.getY());
        addTelemetryData("Robot Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));
        addTelemetryData("Vision Scatter", scatterChoiceToString(currentScatterChoice));
        addIntakeTimingTelemetry();
        addVisionDiagnosticTelemetry();
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

        if (limelight != null) {
            limelight.stop();
        }

        if (follower != null) {
            Pose pose = follower.getPose();
            PoseHandoff.save(pose);
            finalPose = pose;
        }

        AutoFinished = true;

        addTelemetryData("Status", "Stopped");
        addVisionDiagnosticTelemetry();
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

        CollectedScatterA = p(SCATTER_COLLECTED_X, 7.49, 180);
        IntakeScatterA = p(
                SCATTER_COLLECTED_X + SCATTER_COLLECTION_RUN_INCHES,
                7.49,
                180
        );

        IntakeC = p(44, 35, 180);
        CollectedC = p(24, 35, 180);

        CollectedScatterB = p(SCATTER_COLLECTED_X, 21, 180);
        IntakeScatterB = p(
                SCATTER_COLLECTED_X + SCATTER_COLLECTION_RUN_INCHES,
                21,
                180
        );

        CollectedScatterC = p(SCATTER_COLLECTED_X, 32, 180);
        IntakeScatterC = p(
                SCATTER_COLLECTED_X + SCATTER_COLLECTION_RUN_INCHES,
                32,
                180
        );

        ShootPreloadPoint = p(55.1, 13.5, -70);
        ShootAfterTripleC = p(55.1, 13.5, -75.5);
        ShootScatterA = p(55.1, 13.5, -76);
        ShootScatterB = p(55.1, 13.5, -73.6);

        // Explicit point used for the Limelight scan direction.
        VisionScanPoint = p(55.1, 13.5, 180);
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
        return buildOptimizedScatterToCollected(
                shootStart,
                IntakeScatterA,
                CollectedScatterA
        );
    }

    private PathChain buildScatterBToCollected(Pose shootStart) {
        return buildOptimizedScatterToCollected(
                shootStart,
                IntakeScatterB,
                CollectedScatterB
        );
    }

    private PathChain buildScatterCToCollected(Pose shootStart) {
        return buildOptimizedScatterToCollected(
                shootStart,
                IntakeScatterC,
                CollectedScatterC
        );
    }

    /**
     * Builds a smooth scatter collection chain:
     *
     * 1. A cubic Bezier approaches the collection entry point.
     * 2. Its final tangent exactly matches the entry point's 180-degree heading.
     * 3. A straight axial stroke finishes at the unchanged collected endpoint.
     *
     * Matching the Bezier end tangent to the collection line removes the sharp
     * velocity-direction discontinuity that existed between the old two lines.
     */
    private PathChain buildOptimizedScatterToCollected(
            Pose shootStart,
            Pose collectionEntry,
            Pose collectedEndpoint
    ) {
        final double approachDistance = getPathLength(
                shootStart,
                collectionEntry
        );

        final double handleLength = clamp(
                approachDistance * SCATTER_CURVE_HANDLE_RATIO,
                SCATTER_CURVE_MIN_HANDLE_INCHES,
                SCATTER_CURVE_MAX_HANDLE_INCHES
        );

        final Pose startControl = offsetAlongHeading(
                shootStart,
                shootStart.getHeading(),
                handleLength
        );

        /*
         * A cubic Bezier's endpoint tangent points from its second control point
         * toward its endpoint. Placing this control point behind the endpoint's
         * commanded travel direction makes the curve enter collectionEntry at
         * exactly collectionEntry.getHeading().
         */
        final Pose endControl = offsetAlongHeading(
                collectionEntry,
                collectionEntry.getHeading(),
                -handleLength
        );

        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootStart,
                        startControl,
                        endControl,
                        collectionEntry
                ))
                .setHeadingInterpolation(
                        buildDirectShortestTurnHeading(
                                shootStart,
                                collectionEntry,
                                collectionEntry.getHeading(),
                                INTAKE_ALIGN_END_T
                        )
                )

                /*
                 * The final collection stroke is still a pure forward axial line.
                 * Only its start point moved closer to the unchanged endpoint.
                 */
                .addPath(new BezierLine(
                        collectionEntry,
                        collectedEndpoint
                ))
                .setConstantHeadingInterpolation(collectionEntry.getHeading())
                .build();
    }

    private Pose offsetAlongHeading(
            Pose origin,
            double heading,
            double distance
    ) {
        return new Pose(
                origin.getX() + distance * Math.cos(heading),
                origin.getY() + distance * Math.sin(heading),
                heading
        );
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
                    beginVisionScatterSelection();
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
                            returnToShootFromCurrent(currentScatterChoice),
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
                    beginVisionScatterSelection();
                }
                break;

            // Turn in place so the Limelight faces the scatter field.
            case 7:
                if (Math.abs(follower.getHeadingError()) < Math.toRadians(3.0)) {
                    // Capture the current Python reset sequence and heartbeat.
                    // The next decision must come after a newly observed reset
                    // edge and from a newer processed image.
                    peekVisionPipeline("pre-reset baseline");

                    if (lastPythonOutput != null && lastPythonOutput.length >= 7) {
                        observedVisionResetSequence = Math.round(lastPythonOutput[5]);
                        expectedVisionResetSequence = DISABLE_VISION_RESET_FOR_TESTING
                                ? observedVisionResetSequence
                                : observedVisionResetSequence + 1L;
                        scanStartHeartbeat = Math.round(lastPythonOutput[6]);
                    } else {
                        expectedVisionResetSequence = -1L;
                        scanStartHeartbeat = -1L;
                    }

                    sendVisionInputs(!DISABLE_VISION_RESET_FOR_TESTING);
                    setPathState(8);
                }
                break;

            // Reset the Python pipeline, wait briefly for a stable result, then go.
            case 8:
                updateVisionScatterSelection();
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

    private void armIntakeSegmentTimer() {
        watchedScatterChoice = currentScatterChoice;
        watchIntakeSegmentTimer = true;
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

    // =========================================================================
    // Vision diagnostics
    // =========================================================================

    private void resetVisionDiagnostics() {
        lastPythonOutput = null;
        lastVisionReadStatus = "--";
        lastVisionBranch = "--";
        lastVisionStalenessMs = -1L;
        maxPathScoreSeen = 0.0;
        maxPathScoreThisCycle = 0.0;
        visionReadAttempts = 0;
        visionNoResultCount = 0;
        visionStaleCount = 0;
        visionInvalidFlagCount = 0;
        visionAcceptedCount = 0;
        commandedScanHeadingRad = Double.NaN;
        expectedVisionResetSequence = -1L;
        observedVisionResetSequence = -1L;
        scanStartHeartbeat = -1L;
        visionResetNotObservedCount = 0;
        visionOldFrameCount = 0;
        lastPythonFrameHeartbeat = -1L;
        repeatedPythonHeartbeatCount = 0;

        for (int i = 0; i < visionCycleResults.length; i++) {
            visionCycleResults[i] = null;
        }
    }

    /**
     * Latches whatever the pipeline is currently producing, without consuming or
     * acting on it.
     *
     * Called during init_loop (no reset pulse is ever sent there, so it is the
     * clean baseline) and during the post-reset settling window in state 8, so
     * the driver station shows the scores recovering after the pulse rather than
     * only the single frame that happens to fall in the read window.
     */
    private void peekVisionPipeline(String label) {
        if (limelight == null) {
            lastVisionReadStatus = "Limelight is null";
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            lastVisionReadStatus = "No LLResult (" + label + ")";
            lastPythonOutput = null;
            return;
        }

        lastVisionStalenessMs = result.getStaleness();

        if (result.getPipelineIndex() != LIMELIGHT_PIPELINE) {
            lastVisionReadStatus = String.format(
                    Locale.US,
                    "%s — wrong pipeline %d (expected %d)",
                    label,
                    result.getPipelineIndex(),
                    LIMELIGHT_PIPELINE
            );
            lastPythonOutput = null;
            lastVisionBranch = "WRONG PIPELINE";
            return;
        }

        double[] pythonOutput = result.getPythonOutput();

        if (pythonOutput == null) {
            lastVisionReadStatus = "Python output is null (" + label + ")";
            lastPythonOutput = null;
            return;
        }

        latchPythonOutput(pythonOutput);
        lastVisionReadStatus = label;
    }

    private void latchPythonOutput(double[] pythonOutput) {
        lastPythonOutput = pythonOutput.clone();
        lastVisionBranch = describePythonBranch(pythonOutput);

        if (pythonOutput.length >= 6) {
            observedVisionResetSequence = Math.round(pythonOutput[5]);
        }

        if (pythonOutput.length >= 7) {
            final long heartbeat = Math.round(pythonOutput[6]);
            if (heartbeat == lastPythonFrameHeartbeat) {
                repeatedPythonHeartbeatCount++;
            } else {
                lastPythonFrameHeartbeat = heartbeat;
                repeatedPythonHeartbeatCount = 0;
            }
        }

        if (pythonOutput.length >= 5) {
            final double frameMax = Math.max(
                    pythonOutput[2],
                    Math.max(pythonOutput[3], pythonOutput[4])
            );
            if (frameMax > maxPathScoreSeen) {
                maxPathScoreSeen = frameMax;
            }
            if (frameMax > maxPathScoreThisCycle) {
                maxPathScoreThisCycle = frameMax;
            }
        }
    }

    /**
     * Works out which branch of the Python pipeline produced this output.
     *
     * The pipeline returns valid = 0 while a score is below the configured
     * threshold, while a winning path is still being confirmed, or when an
     * exception occurs. Indices 2-4 now always carry CURRENT-FRAME RAW SCORES,
     * including below-threshold values, so Driver Station telemetry matches the
     * raw values shown in the Limelight browser. Index 5 is a Python-generated
     * reset sequence and index 6 is a frame heartbeat that changes every image.
     * Index 7 is zero only for the top-level
     * exception fallback.
     */
    private String describePythonBranch(double[] py) {
        if (py == null) {
            return "NO OUTPUT";
        }

        if (py.length < 8) {
            return "OUTPUT TOO SHORT (" + py.length + " of 8)";
        }

        if (py[0] >= 0.5) {
            return "VALID";
        }

        // The empty/None-image guard returns all zeros, including path 0.
        if (py[1] < 0.5) {
            return "EMPTY IMAGE (pipeline got no frame)";
        }

        if (py[7] <= 0.0) {
            return "PIPELINE EXCEPTION (fps=0) — read the LL console log";
        }

        final double maxScore = Math.max(py[2], Math.max(py[3], py[4]));

        if (maxScore <= 0.0) {
            return "NO COLORED FOREGROUND";
        }

        return "BELOW THRESHOLD OR CONFIRMING";
    }

    private String formatVisionCycleResults() {
        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < visionCycleResults.length; i++) {
            if (i > 0) {
                builder.append(' ');
            }
            builder.append(i + 1).append(':');
            builder.append(visionCycleResults[i] == null ? "--" : visionCycleResults[i]);
        }

        return builder.toString();
    }

    private void recordVisionCycleResult(String result) {
        if (scatterCycleIndex >= 0 && scatterCycleIndex < visionCycleResults.length) {
            visionCycleResults[scatterCycleIndex] = result;
        }
    }

    private void addVisionDiagnosticTelemetry() {
        if (!USE_TELEMETRY) {
            return;
        }

        addTelemetryData("--- VISION ---", "");
        addTelemetryData("Vision Read", lastVisionReadStatus);
        addTelemetryData("Python Branch", lastVisionBranch);

        if (DISABLE_VISION_RESET_FOR_TESTING) {
            addTelemetryData(
                    "Vision Reset",
                    "DISABLED FOR TESTING — do not run a match like this"
            );
        }

        if (!Double.isNaN(commandedScanHeadingRad)) {
            addTelemetryData(
                    "Scan heading cmd",
                    "%.1f deg (%s)",
                    Math.toDegrees(commandedScanHeadingRad),
                    MIRROR_SCAN_HEADING ? "mirrored" : "hardcoded"
            );
        }

        if (lastVisionStalenessMs >= 0L) {
            addTelemetryData(
                    "Vision Staleness",
                    "%d ms (limit %d)",
                    lastVisionStalenessMs,
                    MAX_VISION_STALENESS_MS
            );
        } else {
            addTelemetryData("Vision Staleness", "--");
        }

        double[] py = lastPythonOutput;

        if (py == null) {
            addTelemetryData("Python Output", "none latched yet");
        } else {
            addTelemetryData("Python Length", py.length);

            if (py.length >= 2) {
                addTelemetryData(
                        "Py valid / path",
                        "%.1f / %.1f",
                        py[0],
                        py[1]
                );
            }
            if (py.length >= 5) {
                addTelemetryData(
                        "Py RAW scores P1/P2/P3",
                        "%.5f / %.5f / %.5f",
                        py[2],
                        py[3],
                        py[4]
                );
            }
            if (py.length >= 6) {
                addTelemetryData(
                        "Py reset sequence seen / expected",
                        "%d / %d",
                        Math.round(py[5]),
                        expectedVisionResetSequence
                );
            }
            if (py.length >= 7) {
                addTelemetryData(
                        "Py frame heartbeat / repeats",
                        "%d / %d",
                        Math.round(py[6]),
                        repeatedPythonHeartbeatCount
                );
            }
            if (py.length >= 8) {
                addTelemetryData("Py pipeline FPS", "%.1f", py[7]);
            }
        }

        addTelemetryData(
                "Py max score cycle / run",
                "%.3f / %.3f",
                maxPathScoreThisCycle,
                maxPathScoreSeen
        );
        addTelemetryData(
                "Vision counts att/none/stale/reset/old/inv/ok",
                "%d / %d / %d / %d / %d / %d / %d",
                visionReadAttempts,
                visionNoResultCount,
                visionStaleCount,
                visionResetNotObservedCount,
                visionOldFrameCount,
                visionInvalidFlagCount,
                visionAcceptedCount
        );
        addTelemetryData("Vision cycle results", formatVisionCycleResults());
    }

    // =========================================================================

    private void beginVisionScatterSelection() {
        if (scatterCycleIndex >= SCATTER_CYCLE_COUNT) {
            cancelIntakeSegmentTimer();
            autoManipulator.idle();
            setPathState(-1);
            return;
        }

        autoManipulator.intake();

        // Do NOT call pipelineSwitch() here. Repeated pipeline switches can
        // freeze SnapScript's llpython output even while LLResult timestamps
        // continue changing. The pipeline is selected and recovered once in init.

        maxPathScoreThisCycle = 0.0;

        Pose current = getCurrentPoseOr(ShootAfterTripleC);

        /*
         * MIRROR_SCAN_HEADING selects between the original hardcoded 180 and the
         * FieldMirror-routed VisionScanPoint heading. These are the same value
         * unless FieldMirror mirrors heading; compare the "Scan heading" readouts
         * on both alliances before changing the default.
         */
        commandedScanHeadingRad = MIRROR_SCAN_HEADING
                ? VisionScanPoint.getHeading()
                : Math.toRadians(180.0);

        follower.holdPoint(
                new Pose(
                        current.getX(),
                        current.getY(),
                        commandedScanHeadingRad
                ),
                false
        );

        setPathState(7);
    }

    private void sendVisionInputs(boolean resetRequested) {
        if (limelight == null) {
            return;
        }

        limelight.updatePythonInputs(new double[]{
                resetRequested ? 1.0 : 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        });
    }

    private void updateVisionScatterSelection() {
        final double elapsed = pathTimer.getElapsedTimeSeconds();

        addTelemetryData("Vision Cycle", scatterCycleIndex + 1);
        addTelemetryData("Vision Phase Elapsed", "%.3f sec", elapsed);

        // Pulse llrobot[0] high so the Python pipeline clears its previous result.
        if (!DISABLE_VISION_RESET_FOR_TESTING && elapsed < VISION_RESET_PULSE_SEC) {
            addTelemetryData("Vision Phase", "reset pulse");
            sendVisionInputs(true);
            return;
        }

        sendVisionInputs(false);

        if (elapsed < VISION_READ_DELAY_SEC) {
            addTelemetryData("Vision Phase", "settling after reset");
            // Watch the pipeline recover during the settling window. This is
            // observation only; it cannot start a path.
            peekVisionPipeline("settling peek");
            return;
        }

        addTelemetryData("Vision Phase", "reading");

        int visionChoice = readVisionScatterChoice();

        if (visionChoice >= 0) {
            currentScatterChoice = visionChoice;
            recordVisionCycleResult("P" + (visionChoice + 1));
            startSelectedScatter();
            return;
        }

        if (elapsed >= VISION_TIMEOUT_SEC) {
            /*
             * Previously this printed a message and stayed in state 8 forever,
             * which cost the remainder of autonomous on a single failed read.
             * Falling back to the default scatter keeps the robot scoring: a
             * wrong guess costs one cycle, a freeze costs all of them.
             */
            currentScatterChoice = DEFAULT_SCATTER_CHOICE;
            lastVisionReadStatus = "TIMED OUT — fallback to "
                    + scatterChoiceToString(currentScatterChoice);
            recordVisionCycleResult(
                    "TO->" + scatterChoiceToString(currentScatterChoice)
            );
            addTelemetryData("Vision Read", lastVisionReadStatus);
            startSelectedScatter();
        }
    }

    private int readVisionScatterChoice() {
        visionReadAttempts++;

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            visionNoResultCount++;
            lastVisionReadStatus = "No LLResult";
            lastPythonOutput = null;
            lastVisionBranch = "NO OUTPUT";
            return -1;
        }

        addTelemetryData("Vision Pipeline", result.getPipelineIndex());
        lastVisionStalenessMs = result.getStaleness();

        double[] pythonOutput = result.getPythonOutput();

        if (pythonOutput == null) {
            lastVisionReadStatus = "Python output is null";
            lastPythonOutput = null;
            lastVisionBranch = "NO OUTPUT";
            return -1;
        }

        // Latch everything before any of the rejection checks, so the driver
        // station shows what the pipeline actually said even when it is rejected.
        latchPythonOutput(pythonOutput);

        if (result.getPipelineIndex() != LIMELIGHT_PIPELINE) {
            lastVisionReadStatus = String.format(
                    Locale.US,
                    "Wrong pipeline (%d, expected %d)",
                    result.getPipelineIndex(),
                    LIMELIGHT_PIPELINE
            );
            return -1;
        }

        if (result.getStaleness() > MAX_VISION_STALENESS_MS) {
            visionStaleCount++;
            lastVisionReadStatus = String.format(
                    Locale.US,
                    "Result too old (%d ms)",
                    result.getStaleness()
            );
            return -1;
        }

        if (pythonOutput.length < 7) {
            lastVisionReadStatus = "Output too short for reset sequence/heartbeat";
            return -1;
        }

        final long resultResetSequence = Math.round(pythonOutput[5]);
        final long resultHeartbeat = Math.round(pythonOutput[6]);

        if (!DISABLE_VISION_RESET_FOR_TESTING) {
            if (expectedVisionResetSequence < 0L
                    || resultResetSequence < expectedVisionResetSequence) {
                visionResetNotObservedCount++;
                lastVisionReadStatus = String.format(
                        Locale.US,
                        "Waiting for reset sequence %d (got %d)",
                        expectedVisionResetSequence,
                        resultResetSequence
                );
                return -1;
            }
        }

        if (scanStartHeartbeat >= 0L && resultHeartbeat <= scanStartHeartbeat) {
            visionOldFrameCount++;
            lastVisionReadStatus = String.format(
                    Locale.US,
                    "Waiting for a new Python frame after %d (got %d)",
                    scanStartHeartbeat,
                    resultHeartbeat
            );
            return -1;
        }

        if (pythonOutput[0] < 0.5) {
            visionInvalidFlagCount++;
            lastVisionReadStatus = "Python says invalid";
            return -1;
        }

        int visionPath = (int) Math.round(pythonOutput[1]);

        if (visionPath < 1 || visionPath > 3) {
            lastVisionReadStatus = "Bad path: " + visionPath;
            return -1;
        }

        visionAcceptedCount++;
        lastVisionReadStatus = "Accepted path " + visionPath;

        return visionPath - 1;
    }

    private void startSelectedScatter() {
        armIntakeSegmentTimer();
        follower.followPath(
                getScatterPathToRun(currentScatterChoice),
                1.0,
                true
        );
        setPathState(5);
    }

    private void timeoutBackToShoot() {
        int scatterChoice = watchedScatterChoice >= 0
                ? watchedScatterChoice
                : currentScatterChoice;

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

    private PathChain getScatterPathToRun(int scatterChoice) {
        Pose shootStart = getCurrentPoseOr(ShootAfterTripleC);

        switch (scatterChoice) {
            case 0:
                return buildScatterAToCollected(shootStart);
            case 2:
                return buildScatterCToCollected(shootStart);
            case 1:
            default:
                return buildScatterBToCollected(shootStart);
        }
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
        return currentScatterChoice >= 0 && currentScatterChoice <= 2;
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
}