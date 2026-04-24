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

public abstract class FarHPBase extends OpMode {

    protected abstract Alliance getAlliance();

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static boolean AutoFinished = false;

    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private AutoManipulator autoManipulator;

    private int pathState;

    public static int[] scatterPlan = {1, 1, 1, 1, 1};

    public static Pose finalPose;

    private static final double SHOOTER_VELOCITY = 1930;
    private static final double HP_INTAKE_WAIT = 0.4;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    private Pose startPose;

    private Pose IntakeScatterA;
    private Pose CollectedScatterA;

    private Pose IntakeScatterB;
    private Pose CollectedScatterB;

    private Pose IntakeScatterC;
    private Pose CollectedScatterC;

    private Pose Shoot;
    private Pose Shoot2;
    private Pose Shoot3;

    private Pose IntakeHP;
    private Pose CollectedHP;
    private Pose IntakeHP2;
    private Pose CollectedHP2;
    private Pose HPCornerMid;
    private Pose Out;

    private PathChain ShootPreload;

    private PathChain goToScatterA;
    private PathChain intakeScatterA;
    private PathChain shootScatterA;

    private PathChain goToScatterB;
    private PathChain intakeScatterB;
    private PathChain shootScatterB;

    private PathChain goToScatterC;
    private PathChain intakeScatterC;
    private PathChain shootScatterC;

    private PathChain HP1;
    private PathChain HP2;

    private PathChain ShootHP;
    private PathChain ShootHP2;

    private PathChain GoIn1;
    private PathChain GoOut1;

    private PathChain GoIn2;
    private PathChain GoOut2;
    private PathChain Tran;
    private PathChain Leave;

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

        autoManipulator.setIntakePower(1.0, 1.0);
        autoManipulator.setHoldingPower(1.0, 0.0);
        autoManipulator.setShootingFeedPower(1.0, 1.0);

        buildPaths();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Ready");
        telemetry.addData("Scatter Plan", getScatterPlanString());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Scatter Plan", getScatterPlanString());
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
        telemetry.addData("Scatter Plan", getScatterPlanString());
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autoManipulator.update();
        autonomousPathUpdate();

        shooter.setVelocity(SHOOTER_VELOCITY);
        shooter.update(System.nanoTime());

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scatter Plan", getScatterPlanString());
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

    private void buildPoses() {
        startPose = p(21, 1.5, 90);

        IntakeScatterA = p(25, -5.5, 0);
        CollectedScatterA = p(65, -5.5, 0);

        IntakeScatterB = p(25, -21.5, 0);
        CollectedScatterB = p(65, -21.5, 0);

        IntakeScatterC = p(25, -31.5, 0);
        CollectedScatterC = p(65, -31.5, 0);

        Shoot = p(19.4, -6.5, 111);
        Shoot2 = p(19.4, -6.5, 111);
        Shoot3 = p(19.4, -6.5, 111);

        IntakeHP = p(57, 0.5, 0);
        CollectedHP = p(64, 0.5, 0);

        IntakeHP2 = p(57, -11.5, 45);
        CollectedHP2 = p(62, -1.5, 45);

        HPCornerMid = p(36, -18.5, 120);
        Out = p(40, -0.5, 0);
    }

    private void buildPaths() {
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        goToScatterA = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeScatterA))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeScatterA.getHeading())
                .build();

        intakeScatterA = follower.pathBuilder()
                .addPath(new BezierLine(IntakeScatterA, CollectedScatterA))
                .setLinearHeadingInterpolation(IntakeScatterA.getHeading(), CollectedScatterA.getHeading())
                .build();

        shootScatterA = follower.pathBuilder()
                .addPath(new BezierLine(CollectedScatterA, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterA.getHeading(), Shoot3.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        goToScatterB = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeScatterB))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeScatterB.getHeading())
                .build();

        intakeScatterB = follower.pathBuilder()
                .addPath(new BezierLine(IntakeScatterB, CollectedScatterB))
                .setLinearHeadingInterpolation(IntakeScatterB.getHeading(), CollectedScatterB.getHeading())
                .build();

        shootScatterB = follower.pathBuilder()
                .addPath(new BezierLine(CollectedScatterB, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterB.getHeading(), Shoot3.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        goToScatterC = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeScatterC))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeScatterC.getHeading())
                .build();

        intakeScatterC = follower.pathBuilder()
                .addPath(new BezierLine(IntakeScatterC, CollectedScatterC))
                .setLinearHeadingInterpolation(IntakeScatterC.getHeading(), CollectedScatterC.getHeading())
                .build();

        shootScatterC = follower.pathBuilder()
                .addPath(new BezierLine(CollectedScatterC, Shoot3))
                .setLinearHeadingInterpolation(CollectedScatterC.getHeading(), Shoot3.getHeading())
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

        HP2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, IntakeHP2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), IntakeHP2.getHeading())
                .build();

        GoIn2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, CollectedHP2))
                .setConstantHeadingInterpolation(IntakeHP2.getHeading())
                .build();

        GoOut2 = follower.pathBuilder()
                .addPath(new BezierLine(CollectedHP2, IntakeHP2))
                .setConstantHeadingInterpolation(IntakeHP2.getHeading())
                .build();

        Tran = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, IntakeHP))
                .setLinearHeadingInterpolation(IntakeHP2.getHeading(), IntakeHP.getHeading())
                .build();

        ShootHP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakeHP, HPCornerMid, Shoot2))
                .setLinearHeadingInterpolation(IntakeHP.getHeading(), Shoot2.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        ShootHP2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeHP2, Shoot2))
                .setLinearHeadingInterpolation(IntakeHP2.getHeading(), Shoot2.getHeading())
                .addParametricCallback(0.85, () -> autoManipulator.releaseForShot())
                .build();

        Leave = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, Out))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), Out.getHeading())
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
                if (pathAlmostDone(Shoot.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(2);
                }
                break;

            case 2:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(HP1, true);
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
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(ShootHP, 1.0, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (pathAlmostDone(Shoot2.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(9);
                }
                break;

            case 9:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getGoToScatter(0), true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(getIntakeScatter(0), true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(getShootScatter(0), true);
                    setPathState(12);
                }
                break;

            case 12:
                if (pathAlmostDone(Shoot3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getGoToScatter(1), true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(getIntakeScatter(1), true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(getShootScatter(1), true);
                    setPathState(16);
                }
                break;

            case 16:
                if (pathAlmostDone(Shoot3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(17);
                }
                break;

            case 17:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getGoToScatter(2), true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(getIntakeScatter(2), true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(getShootScatter(2), true);
                    setPathState(20);
                }
                break;

            case 20:
                if (pathAlmostDone(Shoot3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(21);
                }
                break;

            case 21:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getGoToScatter(3), true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(getIntakeScatter(3), true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(getShootScatter(3), true);
                    setPathState(24);
                }
                break;

            case 24:
                if (pathAlmostDone(Shoot3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(25);
                }
                break;

            case 25:
                if (autoManipulator.isShootComplete()) {
                    autoManipulator.intake();
                    follower.followPath(getGoToScatter(4), true);
                    setPathState(26);
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(getIntakeScatter(4), true);
                    setPathState(27);
                }
                break;

            case 27:
                if (!follower.isBusy()) {
                    autoManipulator.hold();
                    follower.followPath(getShootScatter(4), true);
                    setPathState(28);
                }
                break;

            case 28:
                if (pathAlmostDone(Shoot3.getHeading(), HEADING_TOLERANCE_DEG)) {
                    autoManipulator.shoot();
                    setPathState(29);
                }
                break;

            case 29:
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
        double error = Math.atan2(
                Math.sin(follower.getPose().getHeading() - targetHeadingRad),
                Math.cos(follower.getPose().getHeading() - targetHeadingRad)
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

    private PathChain getGoToScatter(int cycleIndex) {
        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return goToScatterA;
            case 2:
                return goToScatterC;
            case 1:
            default:
                return goToScatterB;
        }
    }

    private PathChain getIntakeScatter(int cycleIndex) {
        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return intakeScatterA;
            case 2:
                return intakeScatterC;
            case 1:
            default:
                return intakeScatterB;
        }
    }

    private PathChain getShootScatter(int cycleIndex) {
        switch (getScatterChoiceForCycle(cycleIndex)) {
            case 0:
                return shootScatterA;
            case 2:
                return shootScatterC;
            case 1:
            default:
                return shootScatterB;
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
                + scatterChoiceToString(getScatterChoiceForCycle(3)) + " "
                + scatterChoiceToString(getScatterChoiceForCycle(4));
    }
}