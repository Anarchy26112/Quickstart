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

    private static final long VOLTAGE_IDLE_UPDATE_INTERVAL_NS = 250_000_000L; // 250 ms
    private static final long VOLTAGE_AIM_UPDATE_INTERVAL_NS  = 50_000_000L;  // 50 ms

    private static final double DEFAULT_LOOP_DT_SEC = 0.02;
    private static final double MIN_LOOP_DT_SEC = 0.001;
    private static final double MAX_LOOP_DT_SEC = 0.1;
    private long lastLoopNs = 0L;
    private double loopDtSec = DEFAULT_LOOP_DT_SEC;

    private int pathState;
    private int scatterCycleIndex = 0;

    private static final int SCATTER_CYCLE_COUNT = 7;

    // 0 = Scatter A, 1 = Scatter B, 2 = Scatter C
    //
    public static int[] scatterPlan = {0, 0, 1, 0, 2, 0, 1};
    public static Pose finalPose;

    private static final double SHOOTER_VELOCITY = 1950;
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

    private Pose Shoot;
    private Pose Shoot3;

    private Pose Out;

    private PathChain ShootPreload;
    private PathChain ThirdFull;

    private PathChain scatterAFull;
    private PathChain scatterBFull;
    private PathChain scatterCFull;

    private PathChain Leave;

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

        autoManipulator.setIntakePower(1.0, 0.35);
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

        autonomousPathUpdate();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scatter Cycle", scatterCycleIndex + " / " + SCATTER_CYCLE_COUNT);
        telemetry.addData("Scatter Plan", getScatterPlanString());
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

            // Match TeleOp: shooter motors are expected to be on the non-parent Expansion Hub.
            // If there is only one hub, fall back to that hub.
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

        // Poll only the shooter hub, matching TeleOp.
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

        IntakeScatterA = p(20.5, 8.5, 180);
        CollectedScatterA = p(15.5, 8.5, 180);

        IntakeC = p(44, 35, 180);
        CollectedC = p(15.5, 35, 180);

        IntakeScatterB = p(27, 27, 180);
        CollectedScatterB = p(15.5, 27, 180);

        IntakeScatterC = p(20, 40.7, 180);
        CollectedScatterC = p(15.5, 40.7, 180);

        // Shooting point stays fixed.
        Shoot3 = p(55.1, 13.5, -73);

        Out = p(42, 15, 69.5);
    }

    private void buildPaths() {
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot3))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot3.getHeading())
                .addParametricCallback(0.8, () -> autoManipulator.releaseForShot())
                .build();

        ThirdFull = follower.pathBuilder()
                .addPath(new BezierLine(Shoot3, IntakeC))
                .setLinearHeadingInterpolation(Shoot3.getHeading(), IntakeC.getHeading())

                .addPath(new BezierLine(IntakeC, CollectedC))
                .setConstantHeadingInterpolation(IntakeC.getHeading())

                .addPath(new BezierLine(CollectedC, Shoot3))
                .setLinearHeadingInterpolation(headingFrom(Shoot3, CollectedC), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterAFull = follower.pathBuilder()
                // A is almost straight backward from the fixed shooting point.
                .addPath(new BezierLine(Shoot3, IntakeScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(CollectedScatterA, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterA.getHeading(), Shoot3.getHeading())

                .addParametricCallback(0.60, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterBFull = follower.pathBuilder()
                // B is diagonal from the fixed shooting point, so face closer to the travel direction.
                .addPath(new BezierLine(Shoot3, IntakeScatterB))
                .setLinearHeadingInterpolation(Shoot3.getHeading(), headingFrom(Shoot3, IntakeScatterB))

                .addPath(new BezierLine(IntakeScatterB, CollectedScatterB))
                .setConstantHeadingInterpolation(IntakeScatterB.getHeading())

                // Return mostly backward toward the shooting point, then rotate into shooting heading near the end.
                .addPath(new BezierLine(CollectedScatterB, Shoot3))
                .setLinearHeadingInterpolation(headingFrom(Shoot3, CollectedScatterB), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterCFull = follower.pathBuilder()
                // C is the slowest from the fixed shooting point, but keep support for it if scatterPlan uses 2.
                .addPath(new BezierLine(Shoot3, IntakeScatterC))
                .setLinearHeadingInterpolation(Shoot3.getHeading(), headingFrom(Shoot3, IntakeScatterC))

                .addPath(new BezierLine(IntakeScatterC, CollectedScatterC))
                .setConstantHeadingInterpolation(IntakeScatterC.getHeading())

                .addPath(new BezierLine(CollectedScatterC, CollectedScatterB))
                .setConstantHeadingInterpolation(headingFrom(CollectedScatterC, CollectedScatterB))

                .addPath(new BezierLine(CollectedScatterB, Shoot3))
                .setLinearHeadingInterpolation(headingFrom(Shoot3, CollectedScatterB), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        Leave = follower.pathBuilder()
                .addPath(new BezierLine(Shoot3, Out))
                .setConstantHeadingInterpolation(Shoot3.getHeading())
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
                // After preload shot, go directly into the Intake C / ThirdFull cycle.
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(ThirdFull, 1.0, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Shoot after Intake C / ThirdFull finishes.
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(4);
                }
                break;

            case 4:
                // After the Intake C shot, begin the six scatter cycles.
                if (autoManipulator.isShootComplete()) {
                    scatterCycleIndex = 0;
                    startNextScatterOrLeave();
                }
                break;

            case 5:
                // Scatter path finished. Shoot it.
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(6);
                }
                break;

            case 6:
                // Scatter shot finished. Start the next scatter or leave if all six are complete.
                if (autoManipulator.isShootComplete()) {
                    scatterCycleIndex++;
                    startNextScatterOrLeave();
                }
                break;

            case 100:
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

    private void startNextScatterOrLeave() {
        if (scatterCycleIndex < SCATTER_CYCLE_COUNT) {
            autoManipulator.intake();
            follower.followPath(getScatterFull(scatterCycleIndex), 1.0, true);
            setPathState(5);
        } else {
            follower.followPath(Leave, true);
            setPathState(100);
        }
    }

    private boolean isAutoManipulatorShootingState() {
        switch (pathState) {
            case 2: // preload shot
            case 4: // Intake C / ThirdFull shot
            case 6: // scatter shot
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

    private PathChain getScatterFull(int cycleIndex) {
        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return scatterAFull;
            case 2:
                return scatterCFull;
            case 1:
            default:
                return scatterBFull;
        }
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