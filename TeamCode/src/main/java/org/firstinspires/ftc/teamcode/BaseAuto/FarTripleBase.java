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
    private LynxModule controlHub;

    private double cachedVoltageComp = 1.0;
    private long lastVoltageReadMs = 0L;
    private static final long VOLTAGE_READ_INTERVAL_MS = 1000L;

    private int pathState;

    public static int[] scatterPlan = {0, 1, 0, 1};
    public static Pose finalPose;

    private static final double SHOOTER_VELOCITY = 1950;
    private static final double HP_INTAKE_WAIT = 0.0;
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
    private Pose Shoot2;
    private Pose Shoot3;

    private Pose IntakeHP;
    private Pose CollectedHP;
    private Pose Out;

    private PathChain ShootPreload;

    private PathChain HP1;
    private PathChain GoIn1;
    private PathChain GoOut1;
    private PathChain ShootHP;

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

        autoManipulator.setIntakePower(1.0, 0.7);
        autoManipulator.setHoldingPower(1.0, 0.0);
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
        autonomousPathUpdate();

        shooter.setRobotY(pose.getY());
        shooter.setVelocity(SHOOTER_VELOCITY);
        shooter.update(cachedVoltageComp);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scatter Plan", getScatterPlanString());
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Path Timer", String.format(Locale.US, "%.2f sec", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Voltage Comp", cachedVoltageComp);
        telemetry.addData("Shooter Avg Vel", shooter.getAverageVelocity());
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
        startPose = p(16.2, -8.1, -90);

        IntakeScatterA = p(51.5, -8.5, 180);
        CollectedScatterA = p(57.5, -8.5, 180);

        IntakeC = p(28, -35, 180);
        CollectedC = p(57.5, -35, 180);

        IntakeScatterB = p(45, -27, 180);
        CollectedScatterB = p(57.5, -27, 180);

        IntakeScatterC = p(52, -40.7, 180);
        CollectedScatterC = p(57.5, -40.7, 180);

        Shoot = p(16.9, -13.5, -69.5);
        Shoot2 = p(16.9, -13.5, -69.5);
        Shoot3 = p(16.9, -13.5, -69.5);

        IntakeHP = p(51.5, -8.2, 180);
        CollectedHP = p(59.5, -8.2, 180);

        Out = p(30, -15, -69.5);
    }

    private void buildPaths() {
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        HP1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeHP))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeHP.getHeading())
                .build();

        GoIn1 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP, CollectedHP))
                .setConstantHeadingInterpolation(IntakeHP.getHeading())
                .build();

        GoOut1 = follower.pathBuilder()
                .addPath(new BezierLine(CollectedHP, IntakeHP))
                .setConstantHeadingInterpolation(IntakeHP.getHeading())
                .build();

        ShootHP = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP, Shoot2))
                .setLinearHeadingInterpolation(IntakeHP.getHeading(), Shoot2.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        ThirdFull = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, IntakeC))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), IntakeC.getHeading())

                .addPath(new BezierLine(IntakeC, CollectedC))
                .setLinearHeadingInterpolation(IntakeC.getHeading(), CollectedC.getHeading())

                .addPath(new BezierLine(CollectedC, Shoot3))
                .setLinearHeadingInterpolation(CollectedC.getHeading(), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterAFull = follower.pathBuilder()
                .addPath(new BezierLine(Shoot3, IntakeScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setLinearHeadingInterpolation(IntakeScatterA.getHeading(), CollectedScatterA.getHeading())

                .addPath(new BezierLine(CollectedScatterA, IntakeScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setConstantHeadingInterpolation(IntakeScatterA.getHeading())

                .addPath(new BezierLine(CollectedScatterA, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterA.getHeading(), Shoot3.getHeading())

                .addParametricCallback(0.6, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterBFull = follower.pathBuilder()
                .addPath(new BezierLine(Shoot3, IntakeScatterB))
                .setLinearHeadingInterpolation(Shoot3.getHeading(), IntakeScatterB.getHeading())

                .addPath(new BezierLine(IntakeScatterB, CollectedScatterB))
                .setLinearHeadingInterpolation(IntakeScatterB.getHeading(), CollectedScatterB.getHeading())

                .addPath(new BezierLine(CollectedScatterB, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterB.getHeading(), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        scatterCFull = follower.pathBuilder()
                .addPath(new BezierLine(Shoot3, IntakeScatterC))
                .setConstantHeadingInterpolation(IntakeScatterC.getHeading())

                .addPath(new BezierLine(IntakeScatterC, CollectedScatterC))
                .setLinearHeadingInterpolation(IntakeScatterC.getHeading(), CollectedScatterC.getHeading())

                .addPath(new BezierLine(CollectedScatterC, CollectedScatterB))
                .setConstantHeadingInterpolation(Math.toRadians(90))

                .addPath(new BezierLine(CollectedScatterB, Shoot3))
                .setLinearHeadingInterpolation(Math.toRadians(90), Shoot3.getHeading())

                .addParametricCallback(0.62, () -> autoManipulator.hold())
                .addParametricCallback(0.93, () -> autoManipulator.releaseForShot())
                .build();

        Leave = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, Out))
                .setConstantHeadingInterpolation(Shoot2.getHeading())
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
                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > PRELOAD_SHOOT_DELAY) {
                    autoManipulator.shoot();
                    setPathState(2);
                }
                break;

            case 2:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(HP1, true);
                    setPathState(-2);
                }
                break;

            case -2:
                if (!follower.isBusy()) {
                    follower.followPath(GoIn1, true);
                    setPathState(-3);
                }
                break;

            case -3:
                if (!follower.isBusy()) {
                    follower.followPath(GoOut1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(GoIn1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= HP_INTAKE_WAIT) {
                    follower.followPath(GoOut1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(ShootHP, 1.0, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(7);
                }
                break;

            case 7:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(ThirdFull, 1.0, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(9);
                }
                break;

            case 9:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getScatterFull(0), 1.0, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(11);
                }
                break;

            case 11:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getScatterFull(1), 1.0, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getScatterFull(2), 1.0, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(15);
                }
                break;

            case 15:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getScatterFull(3), 1.0, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    autoManipulator.shoot();
                    setPathState(17);
                }
                break;

            case 17:
                if (autoManipulator.isShootComplete()) {
                    follower.followPath(Leave, true);
                    setPathState(100);
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected Follower getFollower() {
        return follower;
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
        return scatterChoiceToString(getScatterChoiceForCycle(0)) + " "
                + scatterChoiceToString(getScatterChoiceForCycle(1)) + " "
                + scatterChoiceToString(getScatterChoiceForCycle(2)) + " "
                + scatterChoiceToString(getScatterChoiceForCycle(3));
    }
}