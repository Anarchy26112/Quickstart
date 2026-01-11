package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;
import static org.firstinspires.ftc.teamcode.Robot.OperatorControls.POSITIONS_PER_TURN;

import org.firstinspires.ftc.teamcode.Robot.ShooterMacro;
import org.firstinspires.ftc.teamcode.Robot.ShooterMacroGPP; // 21
import org.firstinspires.ftc.teamcode.Robot.ShooterMacroPGP; // 22
import org.firstinspires.ftc.teamcode.Robot.ShooterMacroPPG; // 23
import org.firstinspires.ftc.teamcode.Robot.IntakeMacro;

import java.util.Locale;

@Autonomous(name = "CloseBlueCoordinated", group = "Auto")
public class CloseBlueCoordinated extends OpMode {

    // =========================
    // Pedro Pathing
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // =========================
    // Intake timeout (AUTO)  ✅
    // =========================
    private Timer intakeTimeoutTimer;
    private static final double INTAKE_TIMEOUT_SEC = 3.5;

    // =========================
    // Vision
    // =========================
    private Limelight limelight;
    public static boolean AutoFinished = false;


    /** Stores whichever motif tag we saw first (21/22/23). */
    private int motifTagId = -1;

    // =========================
    // Subsystems
    // =========================
    private Intake intake;
    private SpinDex spinDex;
    private Shooter shooter;
    private Pusher pusher;
    private ColorSensor colorSensor;

    public ShooterMacro shooterMacro;
    public ShooterMacroGPP shooterMacroGPP;
    public ShooterMacroPGP shooterMacroPGP;
    public ShooterMacroPPG shooterMacroPPG;
    public IntakeMacro intakeMacro;

    // =========================
    // Shooter macro gating fix ✅
    // =========================
    private enum ActiveShooterMacro {
        NONE,
        DEFAULT,
        GPP,
        PGP,
        PPG
    }
    private ActiveShooterMacro activeShooterMacro = ActiveShooterMacro.NONE;

    // =========================
    // State tracking
    // =========================
    private int pathState;

    // =========================
    // Starting pose + path points
    // =========================
    private final Pose startPose = new Pose(119, 45, Math.toRadians(-128.5)); // close start, used to be 175,23, Math.toRadians(-135)

    public static Pose finalPose;

    // Shooting / look points
    private final Pose angle32Pt = new Pose(80, 19, Math.toRadians(46.5));
    private final Pose lookTag   = new Pose(80, 19, Math.toRadians(-20));

    // Optional "outshot" waypoints (were commented before; now built to avoid NPE)
    private final Pose OutShotZone  = new Pose(72, 20, Math.toRadians(90));
    private final Pose OutShotZone2 = new Pose(48, 20, Math.toRadians(90));

    // Collect points
    private final Pose startPt = new Pose(119, 45, Math.toRadians(-135));
    private final Pose firstTripleCollect      = new Pose(72, 31, Math.toRadians(90));
    private final Pose CollectedFirstTriple    = new Pose(72, 54, Math.toRadians(90));
    private final Pose secondTripleCollect     = new Pose(48, 31, Math.toRadians(90));
    private final Pose CollectedSecondTriple   = new Pose(48, 54, Math.toRadians(90));
    private final Pose PushGatePt = new Pose(67,56, Math.toRadians(0));
    private final Pose EndPoint = new Pose(70,39, Math.toRadians(90));

    // Paths
    private PathChain parkoutsideshooting, parkoutsideshooting2, parkoutsideshooting3;
    private PathChain angle32, LookAtAprilTag, PushGate, PushGateShoot;
    private PathChain goTocollectFirstTriple, IntakeFirstTriple, ShootFirstTriple;
    private PathChain goTocollectSecondTriple, IntakeSecondTriple, ShootSecondTriple;

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Intake timeout timer ✅
        intakeTimeoutTimer = new Timer();
        intakeTimeoutTimer.resetTimer();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("Follower", "Initialized");

        // Subsystems
        intake = new Intake(hardwareMap, telemetry);
        spinDex = new SpinDex(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);

        // Preload spindex
        spinDex.setSlot(0, SpinDex.ArtifactType.GREEN);
        spinDex.setSlot(1, SpinDex.ArtifactType.PURPLE);
        spinDex.setSlot(2, SpinDex.ArtifactType.PURPLE);

        pusher.stop();

        shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);
        shooterMacroGPP = new ShooterMacroGPP(spinDex, shooter, pusher, telemetry);
        shooterMacroPGP = new ShooterMacroPGP(spinDex, shooter, pusher, telemetry);
        shooterMacroPPG = new ShooterMacroPPG(spinDex, shooter, pusher, telemetry);

        intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, shooter, telemetry);

        telemetry.addData("Subsystems", "Initialized");

        // Vision
        limelight = new Limelight(hardwareMap, telemetry);
        limelight.setTargetMotif(); // only look for 21/22/23

        // Paths
        buildPaths();
        telemetry.addData("Paths", "Built");

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        limelight.update();
        if (limelight.isTargetVisible()) {
            motifTagId = limelight.getDetectedTagId();
        }

        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

        telemetry.addData("Motif Scan", limelight.isTargetVisible() ? "FOUND" : "searching...");
        telemetry.addData("Motif Tag ID", motifTagId);

        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        // ✅ FIX: don't start in STOP state (-1). Start by moving to lookTag then scan.
        //✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅
        setPathState(-3);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Update macros/subsystems
        intakeMacro.update();
        spinDex.periodic();
        pusher.update();

        shooterMacro.update();
        shooterMacroGPP.update();
        shooterMacroPGP.update();
        shooterMacroPPG.update();

        // Run auto state machine
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

        telemetry.addData("Motif Tag ID", motifTagId);
        telemetry.addData("Active Shooter Macro", activeShooterMacro);

        telemetry.addData("Intake Running?", intakeMacro.isRunning());
        telemetry.addData("Intake Timeout (s)", String.format(Locale.US, "%.2f", intakeTimeoutTimer.getElapsedTimeSeconds()));

        if (intakeMacro.isRunning()) intakeMacro.addTelemetry();

        if (shooterMacro.isRunning()) shooterMacro.addTelemetry();
        if (shooterMacroGPP.isRunning()) shooterMacroGPP.addTelemetry();
        if (shooterMacroPGP.isRunning()) shooterMacroPGP.addTelemetry();
        if (shooterMacroPPG.isRunning()) shooterMacroPPG.addTelemetry();

        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("Active Shooter Running?", isActiveShooterMacroRunning());
        telemetry.addData("SpinDex Empty?", spinDex.isEmpty());

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        int currentPos = spinDex.getCurrentPosition();
        int turn = spinDex.getCurrentTurn();
        int posInTurn = currentPos % POSITIONS_PER_TURN;

        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Turn", turn);
        telemetry.addData("posInTurn", posInTurn);

        telemetry.update();

        // (Optional) keep shooter spun
        shooter.setVelocity(1980.0);
    }

    @Override
    public void stop() {
        intake.stop();
        shooter.stop();
        pusher.stop();
        if (limelight != null) limelight.stop();

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // =========================
    // SHOOTER MACRO PICKER ✅
    // =========================

    private void stopAllShooterMacros() {
        shooterMacro.stop();
        shooterMacroGPP.stop();
        shooterMacroPGP.stop();
        shooterMacroPPG.stop();
        activeShooterMacro = ActiveShooterMacro.NONE;
    }

    private void startCorrectShooterMacro(double velocity) {
        stopAllShooterMacros();

        // If not full, use default macro
        if (!spinDex.hasTwoPurplesOneGreen()) {
            shooterMacro.start(velocity);
            activeShooterMacro = ActiveShooterMacro.DEFAULT;
            return;
        }

        // Full: choose based on tag
        if (motifTagId == 21) {
            shooterMacroGPP.start(velocity);
            activeShooterMacro = ActiveShooterMacro.GPP;
        } else if (motifTagId == 22) {
            shooterMacroPGP.start(velocity);
            activeShooterMacro = ActiveShooterMacro.PGP;
        } else if (motifTagId == 23) {
            shooterMacroPPG.start(velocity);
            activeShooterMacro = ActiveShooterMacro.PPG;
        } else {
            shooterMacro.start(velocity);
            activeShooterMacro = ActiveShooterMacro.DEFAULT;
        }
    }

    private boolean isActiveShooterMacroComplete() {
        switch (activeShooterMacro) {
            case DEFAULT: return shooterMacro.isComplete();
            case GPP:     return shooterMacroGPP.isComplete();
            case PGP:     return shooterMacroPGP.isComplete();
            case PPG:     return shooterMacroPPG.isComplete();
            case NONE:
            default:      return false;
        }
    }

    private boolean isActiveShooterMacroRunning() {
        switch (activeShooterMacro) {
            case DEFAULT: return shooterMacro.isRunning();
            case GPP:     return shooterMacroGPP.isRunning();
            case PGP:     return shooterMacroPGP.isRunning();
            case PPG:     return shooterMacroPPG.isRunning();
            case NONE:
            default:      return false;
        }
    }

    // =========================
    // PATH BUILDING
    // =========================
    public void buildPaths() {

        // Drive to a look pose (so the camera can see tags reliably)
        LookAtAprilTag = follower.pathBuilder()
                .addPath(new BezierLine(startPt, lookTag))
                .setLinearHeadingInterpolation(startPt.getHeading(), lookTag.getHeading())
                .addTemporalCallback(0, () -> shooter.setVelocity(1980))
                .addTemporalCallback(0, () -> spinDex.moveToPosition(3))
                .build();

        PushGate = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, PushGatePt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), PushGatePt.getHeading())
                .build();

        PushGateShoot = follower.pathBuilder()
                .addPath(new BezierLine(PushGatePt, angle32Pt))
                .setLinearHeadingInterpolation(PushGatePt.getHeading(), angle32Pt.getHeading())
                .build();


        // Move to shooting angle
        angle32 = follower.pathBuilder()
                .addPath(new BezierLine(startPt, angle32Pt))
                .setLinearHeadingInterpolation(startPt.getHeading(), angle32Pt.getHeading())
                .addTemporalCallback(1, () -> shooter.setVelocity(0))
                .build();

        // Outshot zones (built so state machine never hits null)
        parkoutsideshooting = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone.getHeading())
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();

        parkoutsideshooting2 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone2))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone2.getHeading())
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();

        parkoutsideshooting3 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, EndPoint))
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), EndPoint.getHeading())
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();

        goTocollectFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone, firstTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone.getHeading(), firstTripleCollect.getHeading())
                .build();

        goTocollectSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone2, secondTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone2.getHeading(), secondTripleCollect.getHeading())
                .build();

        IntakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(firstTripleCollect, CollectedFirstTriple))
                .setLinearHeadingInterpolation(firstTripleCollect.getHeading(), CollectedFirstTriple.getHeading())
                .build();

        IntakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(secondTripleCollect, CollectedSecondTriple))
                .setLinearHeadingInterpolation(secondTripleCollect.getHeading(), CollectedSecondTriple.getHeading())
                .build();

        ShootFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())
                .build();

        ShootSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), angle32Pt.getHeading())
                .build();
    }

    // =========================
    // AUTO STATE MACHINE ✅
    // - scan motif in -2 (after driving to look pose in -3)
    // - shooter: use picker + active macro completion
    // - intake: timeout in cases 5 and 10
    // =========================
    public void autonomousPathUpdate() {
        switch (pathState) {

            // Drive to look pose
            case -3: {
                follower.followPath(LookAtAprilTag, true);
                setPathState(-2);
                break;
            }

            // Scan motif and wait until visible (same behavior as your far auto)
            case -2: {
                limelight.update();

                if (limelight.isTargetVisible()) {
                    motifTagId = limelight.getDetectedTagId();
                    setPathState(0);
                }

                telemetry.addData("Scanning Motif", "true");
                telemetry.addData("Motif Visible", limelight.isTargetVisible());
                telemetry.addData("Motif Tag ID", motifTagId);
                break;
            }

            // Go to shooting angle
            case 0:
                follower.followPath(angle32, true);
                setPathState(1);
                break;

            // Shoot preload using picker ✅
            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(1980);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        if (isActiveShooterMacroComplete()) {
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(2);
                        }
                    }
                }
                break;

            // Move out then to collect first triple
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectFirstTriple,0.7,true);
                    setPathState(4);
                }
                break;

            // Start intake + reset timeout clock ✅
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeFirstTriple, 0.35, true);
                    if (!intakeMacro.isRunning() && !spinDex.isFull()) {
                        intakeMacro.start();
                        intakeTimeoutTimer.resetTimer();
                    }
                    setPathState(5);
                }
                break;

            // Intake timeout handling ✅
            case 5:
                if (!follower.isBusy()) {
                    boolean timedOut = intakeMacro.isRunning()
                            && intakeTimeoutTimer.getElapsedTimeSeconds() >= INTAKE_TIMEOUT_SEC;

                    if (timedOut) {
                        intakeMacro.stop();
                    }

                    if (!intakeMacro.isRunning() || timedOut) {
                        follower.followPath(PushGate);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(PushGateShoot);
                    setPathState(7);
                }

            // Shoot after first triple using picker ✅
            case 7:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(1980);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                        if (isActiveShooterMacroComplete()) {
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(8);
                        }
                    }
                }
                break;

            // Move out to second outshot zone, then collect second triple
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting2);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectSecondTriple,0.7,true);
                    setPathState(10);
                }
                break;

            // Start intake 2 + reset timeout clock ✅
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeSecondTriple, 0.35, true);
                    if (!intakeMacro.isRunning() && !spinDex.isFull()) {
                        intakeMacro.start();
                        intakeTimeoutTimer.resetTimer();
                    }
                    setPathState(11);
                }
                break;

            // Intake timeout 2 ✅
            case 11:
                if (!follower.isBusy()) {
                    boolean timedOut = intakeMacro.isRunning()
                            && intakeTimeoutTimer.getElapsedTimeSeconds() >= INTAKE_TIMEOUT_SEC;

                    if (timedOut) {
                        intakeMacro.stop();
                    }

                    if (!intakeMacro.isRunning() || timedOut) {
                        follower.followPath(ShootSecondTriple);
                        setPathState(12);
                    }
                }
                break;

            // Shoot after second triple using picker ✅
            case 12:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(1980);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                        if (isActiveShooterMacroComplete()) {
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(13);
                        }
                    }
                }
                break;

            // Park / end
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting3);
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                // Done
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