package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.google.android.gms.auth.api.signin.GoogleSignInAccount;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "FarBlueAuto", group = "Auto")
public class FarBlueAuto extends OpMode {

    // =========================
    // Pedro Pathing
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // =========================
    // Intake timeout (AUTO)
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
    // Shooter macro gating fix:
    // Track which macro we started most recently
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
    private final Pose startPose = new Pose(0, 16, Math.toRadians(0)); //ADD 40 TO ALL Y VALUES

    public static Pose finalPose;

    private Pose OutShotZone = new Pose(22, 16, Math.toRadians(90));
    private Pose OutShotZone2 = new Pose(44.35, 16, Math.toRadians(90));
    private Pose angle32Pt = new Pose(8, 16, Math.toRadians(22));
    private Pose startPt = new Pose(0, 16, Math.toRadians(0));
    private Pose firstTripleCollect = new Pose(26, 27, Math.toRadians(90));
    private Pose CollectedFirstTriple = new Pose(26, 57, Math.toRadians(90));
    private Pose secondTripleCollect = new Pose(50.35, 27, Math.toRadians(90));
    private Pose CollectedSecondTriple = new Pose(50.35, 57, Math.toRadians(90));
    private Pose GoingBackMid = new Pose(20,26,Math.toRadians(22));
    private Pose EndPoint = new Pose(26,16, Math.toRadians(-110));

    private PathChain parkoutsideshooting, angle32, goTocollectFirstTriple, IntakeFirstTriple, ShootFirstTriple,
            parkoutsideshooting2, parkoutsideshooting3, goTocollectSecondTriple, IntakeSecondTriple, ShootSecondTriple, SecondMid;

    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Intake timeout timer
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

        // For auto we need the spindex to know its preloaded
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
        // Keep scanning during init for debug
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

        if (intakeMacro.isRunning()) {
            intakeMacro.addTelemetry();
        }

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

        // FIRST THING: scan motif and store the id, then continue
        setPathState(-2);

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

        // Intake timeout telemetry (optional but handy)
        telemetry.addData("Intake Running?", intakeMacro.isRunning());
        telemetry.addData("Intake Timeout (s)", String.format(Locale.US, "%.2f", intakeTimeoutTimer.getElapsedTimeSeconds()));

        if (intakeMacro.isRunning()) {
            intakeMacro.addTelemetry();
        }

        // Shooter telemetry: show whichever is active (and also the default if running)
        if (shooterMacro.isRunning()) shooterMacro.addTelemetry();
        if (shooterMacroGPP.isRunning()) shooterMacroGPP.addTelemetry();
        if (shooterMacroPGP.isRunning()) shooterMacroPGP.addTelemetry();
        if (shooterMacroPPG.isRunning()) shooterMacroPPG.addTelemetry();

        telemetry.addData("SLOTS (Count: %d)", spinDex.getFilledCount());
        telemetry.addData("Slot 0", spinDex.getSlot(0));
        telemetry.addData("Slot 1", spinDex.getSlot(1));
        telemetry.addData("Slot 2", spinDex.getSlot(2));

        telemetry.addData("═══ CASE 6 DEBUG ═══", "");
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.addData("Active Shooter Running?", isActiveShooterMacroRunning());
        telemetry.addData("SpinDex Empty?", spinDex.isEmpty());

        telemetry.addData("", "");
        telemetry.addData("SpinDex At Target?", spinDex.isAtTarget());
        telemetry.addData("Distance to Target", "%.1f ticks",
                Math.abs(spinDex.getTargetPositionTicks() - spinDex.getMotorPosition()));
        telemetry.addData("Current Ticks", spinDex.getMotorPosition());
        telemetry.addData("Target Ticks", "%.1f", spinDex.getTargetPositionTicks());

        telemetry.addData("Color L", colorSensor.getDetailedColorInfoL());
        telemetry.addData("Color R", colorSensor.getDetailedColorInfoR());

        int currentPos = spinDex.getCurrentPosition();
        int turn = spinDex.getCurrentTurn();
        int posInTurn = currentPos % POSITIONS_PER_TURN;

        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Turn", turn);
        telemetry.addData("posInTurn", posInTurn);

        telemetry.update();

        shooter.setVelocity(2245.0);
    }

    @Override
    public void stop() {
        // Emergency stop all subsystems
        intake.stop();
        shooter.stop();
        pusher.stop();

        if (limelight != null) limelight.stop();

        AutoFinished = true;

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // =========================
    // SHOOTER MACRO PICKER (fixed)
    // =========================

    /** Stop/reset all shooter macros so stale COMPLETE flags cannot interfere. */
    private void stopAllShooterMacros() {
        shooterMacro.stop();

        shooterMacroGPP.stop();
        shooterMacroPGP.stop();
        shooterMacroPPG.stop();

        activeShooterMacro = ActiveShooterMacro.NONE;
    }

    /** Start the correct shooter macro and remember which one we started. */
    private void startCorrectShooterMacro(double velocity) {
        // Clear stale state before starting a new volley
        stopAllShooterMacros();

        if (!spinDex.hasTwoPurplesOneGreen()) {
            shooterMacro.start(velocity);
            activeShooterMacro = ActiveShooterMacro.DEFAULT;
            return;
        }

        // If we ARE full, use the tag-based macro
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
            // Fallback if tag is missing
            shooterMacro.start(velocity);
            activeShooterMacro = ActiveShooterMacro.DEFAULT;
        }
    }

    /** Only check completion of the macro we actually started. */
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
        angle32 = follower.pathBuilder()
                .addPath(new BezierLine(startPt, angle32Pt))
                .setLinearHeadingInterpolation(startPt.getHeading(), angle32Pt.getHeading())
                //.setVelocityConstraint(0.025)
                //.setBrakingStrength(2)
                .addTemporalCallback(0, () -> shooter.setVelocity(2245.0))
                .addTemporalCallback(0, () -> spinDex.moveToPosition(3))
                .build();

        parkoutsideshooting = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone))
                //.setVelocityConstraint(0.025)
               // .setBrakingStrength(2)
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone.getHeading())
                .build();

        parkoutsideshooting3 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, EndPoint))
                //.setVelocityConstraint(0.025)
                // .setBrakingStrength(2)
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), EndPoint.getHeading())
                .build();

        SecondMid = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, GoingBackMid))
                .setConstantHeadingInterpolation(0)
                .build();

        parkoutsideshooting2 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone2))
                //.setVelocityConstraint(0.025)
                //.setBrakingStrength(2)
                .setLinearHeadingInterpolation(angle32Pt.getHeading(), OutShotZone2.getHeading())
                .build();

        goTocollectFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone, firstTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone.getHeading(), firstTripleCollect.getHeading())
               // .setVelocityConstraint(0.025)
               // .setBrakingStrength(2)
                .build();

        goTocollectSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone2, secondTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone2.getHeading(), secondTripleCollect.getHeading())
              //  .setVelocityConstraint(0.025)
              //  .setBrakingStrength(2)
                .build();

        IntakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(firstTripleCollect, CollectedFirstTriple))
                .setLinearHeadingInterpolation(firstTripleCollect.getHeading(), CollectedFirstTriple.getHeading())
              //  .setVelocityConstraint(0.025)
               // .setBrakingStrength(2)
                .build();

        IntakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(secondTripleCollect, CollectedSecondTriple))
                .setLinearHeadingInterpolation(secondTripleCollect.getHeading(), CollectedSecondTriple.getHeading())
             //   .setVelocityConstraint(0.025)
             //   .setBrakingStrength(2)
                .build();

        ShootFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, GoingBackMid))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())
                .addPath(new BezierLine(GoingBackMid, angle32Pt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())

                // .setVelocityConstraint(0.025)
               // .setBrakingStrength(2)
                .build();

        ShootSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, GoingBackMid))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), angle32Pt.getHeading())
                .addPath(new BezierLine(GoingBackMid, angle32Pt))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), angle32Pt.getHeading())
               // .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();
    }

    // =========================
    // AUTO STATE MACHINE
    // =========================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case -2: {
                limelight.update();

                // Guaranteed motif exists -> wait here until we see it
                // (You may want to add a timeout fallback, but leaving as-is per your original design.)
                if (limelight.isTargetVisible()) {
                    motifTagId = limelight.getDetectedTagId();
                    setPathState(0); // continue into normal auto
                }

                telemetry.addData("Scanning Motif", "true");
                telemetry.addData("Motif Visible", limelight.isTargetVisible());
                telemetry.addData("Motif Tag ID", motifTagId);
                break;
            }

            case 0:
                follower.followPath(angle32, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(2245.0);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        if (isActiveShooterMacroComplete()) {
                            // Clear active macro selection so future volleys are clean
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectFirstTriple, 0.7, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeFirstTriple, 0.35, true);
                    if (!intakeMacro.isRunning() && !spinDex.isFull()) {
                        intakeMacro.start();
                        intakeTimeoutTimer.resetTimer(); // <-- start timeout clock
                    }
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    boolean timedOut = intakeMacro.isRunning()
                            && intakeTimeoutTimer.getElapsedTimeSeconds() >= INTAKE_TIMEOUT_SEC;

                    if (timedOut) {
                        intakeMacro.stop(); // <-- stop intake if timed out
                    }

                    if (!intakeMacro.isRunning() || timedOut) {
                        follower.followPath(ShootFirstTriple);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(2245.0);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                        if (isActiveShooterMacroComplete()) {
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(7);
                        }
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting2);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(goTocollectSecondTriple);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeSecondTriple, 0.35, true);
                    if (!intakeMacro.isRunning() && !spinDex.isFull()) {
                        intakeMacro.start();
                        intakeTimeoutTimer.resetTimer(); // <-- start timeout clock
                    }
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    boolean timedOut = intakeMacro.isRunning()
                            && intakeTimeoutTimer.getElapsedTimeSeconds() >= INTAKE_TIMEOUT_SEC;

                    if (timedOut) {
                        intakeMacro.stop();
                    }

                    if (!intakeMacro.isRunning() || timedOut) {
                        follower.followPath(ShootSecondTriple,true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        if (!spinDex.isEmpty() && !isActiveShooterMacroRunning()) {
                            startCorrectShooterMacro(2245.0);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                        if (isActiveShooterMacroComplete()) {
                            activeShooterMacro = ActiveShooterMacro.NONE;
                            setPathState(12);
                        }
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting3,true);
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
