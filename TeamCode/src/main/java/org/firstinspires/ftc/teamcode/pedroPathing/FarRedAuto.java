package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.*;
import org.firstinspires.ftc.teamcode.Robot.ShooterMacro;
import org.firstinspires.ftc.teamcode.Robot.IntakeMacro;

import java.util.Locale;

@Autonomous(name = "FarRedAuto", group = "Auto")
public class FarRedAuto extends OpMode {

    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private boolean goSlow = true;

    // Subsystems
    private Intake intake;
    private SpinDex spinDex;
    private Shooter shooter;
    private Pusher pusher;
    private ColorSensor colorSensor;

    public ShooterMacro shooterMacro;
    public IntakeMacro intakeMacro;

    // State tracking
    private int pathState;

    // Starting pose
    private final Pose startPose = new Pose(0, 24, Math.toRadians(0)); //used to be 52,7 i shifted coords to center it better,,, x-52 and y+17
    public static Pose finalPose;
    private Pose OutShotZone = new Pose(20, 24, Math.toRadians(0));
    private Pose OutShotZone2 = new Pose(44.35,24,Math.toRadians(0));
    private Pose angle32Pt = new Pose(8,24, Math.toRadians(-22));
    private Pose startPt = new Pose(0, 24, Math.toRadians(0));
    private Pose firstTripleCollect = new Pose(26, 15, Math.toRadians(-90));
    private Pose CollectedFirstTriple = new Pose(26, -8, Math.toRadians(-90));
    private Pose secondTripleCollect = new Pose(50.35, 15, Math.toRadians(-90));
    private Pose CollectedSecondTriple = new Pose(50.35, -8, Math.toRadians(-90));
    private PathChain parkoutsideshooting, angle32, goTocollectFirstTriple, IntakeFirstTriple, ShootFirstTriple, parkoutsideshooting2, goTocollectSecondTriple, IntakeSecondTriple, ShootSecondTriple;

    // chamber scoring positions, all slightly different
    double heading = Math.toRadians(0);
    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize telemetry
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("Follower", "Initialized");

        // Initialize all subsystems
        intake = new Intake(hardwareMap, telemetry);
        spinDex = new SpinDex(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);


        //for auto we need the spindex to know its preloaded
        spinDex.setSlot(0, SpinDex.ArtifactType.PURPLE);
        spinDex.setSlot(1, SpinDex.ArtifactType.PURPLE);
        spinDex.setSlot(2, SpinDex.ArtifactType.GREEN);
        pusher.stop();
        telemetry.addData("Subsystems", "Initialized");
        shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);
        intakeMacro = new IntakeMacro(intake, spinDex, colorSensor, shooter, telemetry);

        // Build paths
        buildPaths();
        telemetry.addData("Paths", "Built");

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Display status while waiting for start
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0); //set to 0 for regular auto and -1 for telemetry position
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Pedro Pathing follower
        follower.update();

        // Update pusher state machine (required every loop)
        pusher.update();
        intakeMacro.update();
        shooterMacro.update();

        // Run autonomous path updates
        autonomousPathUpdate();

        // Telemetry feedback
        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");
        /*intakeMacro.update();*/
        telemetry.update();
    }

    @Override
    public void stop() {
        // Emergency stop all subsystems
        intake.stop();
        shooter.stop();
        pusher.stop();

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    //  PATH BUILDING
    public void buildPaths() {
        angle32 = follower.pathBuilder()
                .addPath(new BezierLine(startPt, angle32Pt))
                .setLinearHeadingInterpolation(startPt.getHeading(), angle32Pt.getHeading())

                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                //.addTemporalCallback(0, () -> pusher.push())
                //.addTemporalCallback(0.5, () -> pusher.stop())
                .addTemporalCallback(1, () -> shooter.setVelocity(0))
                .build();

        parkoutsideshooting = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone))
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .setConstantHeadingInterpolation(0)
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();

        parkoutsideshooting2 = follower.pathBuilder()
                .addPath(new BezierLine(angle32Pt, OutShotZone2))
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .setConstantHeadingInterpolation(0)
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();


        goTocollectFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone, firstTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone.getHeading(), firstTripleCollect.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();

        goTocollectSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(OutShotZone2, secondTripleCollect))
                .setLinearHeadingInterpolation(OutShotZone2.getHeading(), secondTripleCollect.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();


        IntakeFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(firstTripleCollect, CollectedFirstTriple))
                .setLinearHeadingInterpolation(firstTripleCollect.getHeading(), CollectedFirstTriple.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                //.addTemporalCallback(0, () -> intakeMacro.start())
                .build();

        IntakeSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(secondTripleCollect, CollectedSecondTriple))
                .setLinearHeadingInterpolation(secondTripleCollect.getHeading(), CollectedSecondTriple.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                //.addTemporalCallback(0, () -> intakeMacro.start())
                .build();

        ShootFirstTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedFirstTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedFirstTriple.getHeading(), angle32Pt.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();

        ShootSecondTriple = follower.pathBuilder()
                .addPath(new BezierLine(CollectedSecondTriple, angle32Pt))
                .setLinearHeadingInterpolation(CollectedSecondTriple.getHeading(), angle32Pt.getHeading())
                .setVelocityConstraint(0.025)
                .setBrakingStrength(2)
                .build();

    }


    //  PATH STATE MACHINE

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(angle32, true);
                //use maxpower to set speed
                setPathState(1);
                //}
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        if (!shooterMacro.isRunning() && !spinDex.isEmpty()) {
                            shooterMacro.start(2200.00);
                        }
                        //shooterMacro.update();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(2);
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
                    follower.followPath(goTocollectFirstTriple);
                    setPathState(4);
                }
                break;

                case 4:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeFirstTriple, 0.25, true);
                        if (!intakeMacro.isRunning() && spinDex.isEmpty()) {
                            intakeMacro.start();
                        }
                        //intakeMacro.update();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && !intakeMacro.isRunning()) {
                    follower.followPath(ShootFirstTriple);
                    setPathState(6);
                }
                break;

                case 6:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        if (!shooterMacro.isRunning() && !spinDex.isEmpty()) {
                            shooterMacro.start(2200.00);
                        }
                        //shooterMacro.update();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()){
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
                    follower.followPath(IntakeSecondTriple, 0.25, true);
                    if (!intakeMacro.isRunning() && spinDex.isEmpty()) {
                        intakeMacro.start();
                    }
                    //intakeMacro.update();
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && !intakeMacro.isRunning()) {
                    follower.followPath(ShootSecondTriple);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        if (!shooterMacro.isRunning() && !spinDex.isEmpty()) {
                            shooterMacro.start(2200.00);
                        }
                        //shooterMacro.update();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(parkoutsideshooting);
                    setPathState(-1);
                }
                break;
                // Add more cases as needed for your autonomous routine
            // ...

            case -1:
            default:
                // Stop state - autonomous complete, do nothing
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
