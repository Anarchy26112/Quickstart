package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;
import org.firstinspires.ftc.teamcode.Robot.ShooterMacro;

import java.util.Locale;

@Autonomous(name = "New Test", group = "Auto")
public class FarAutoNAColor extends OpMode {

    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // Subsystems
    private Intake intake;
    private SpinDex spinDex;
    private Shooter shooter;
    private Pusher pusher;

    public ShooterMacro shooterMacro;


    // State tracking
    private int pathState;

    // Starting pose
    private final Pose startPose = new Pose(52, 7, Math.toRadians(0));
    public static Pose finalPose;
    private Pose Pt1 = new Pose(72, 7);
    private Pose startPt = new Pose(52, 7);
    private PathChain parkoutsideshooting, test2;

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


        //for auto we need the spindex to know its preloaded
        spinDex.setSlot(0, SpinDex.ArtifactType.PURPLE);
        spinDex.setSlot(1, SpinDex.ArtifactType.PURPLE);
        spinDex.setSlot(2, SpinDex.ArtifactType.GREEN);
        pusher.stop();
        telemetry.addData("Subsystems", "Initialized");
        shooterMacro = new ShooterMacro(spinDex, shooter, pusher, telemetry);

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
        setPathState(1);
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Pedro Pathing follower
        follower.update();

        // Update pusher state machine (required every loop)
        pusher.update();

        // Run autonomous path updates
        autonomousPathUpdate();

        // Telemetry feedback
        telemetry.addData("Path State", pathState);
        telemetry.addData("Runtime", String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("", "");

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
        parkoutsideshooting = follower.pathBuilder()
                .addPath(new BezierLine(startPt, Pt1))
                .setConstantHeadingInterpolation(0)
                .addTemporalCallback(0, () -> shooter.setVelocity(0))
                .build();

        test2 = follower.pathBuilder()
                .addPath(new BezierLine(Pt1, startPt))
                .setConstantHeadingInterpolation(0)
                //.addTemporalCallback(0, () -> pusher.push())
                //.addTemporalCallback(0.5, () -> pusher.stop())
                .addTemporalCallback(1, () -> shooter.setVelocity(0))
                .build();

    }


    //  PATH STATE MACHINE

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                break;

            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        if (!shooterMacro.isRunning() && !spinDex.isEmpty()) {
                            shooterMacro.start(2200.00);
                        }
                        shooterMacro.update();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 8.0) {
                        setPathState(2);
                    }
                }
                break;

            case 2:
                follower.followPath(parkoutsideshooting, true);
                //if (!follower.isBusy()) {
                setPathState(-1);
                //}

                /* if (!follower.isBusy()) {
                    follower.followPath(test2, true);
                    setPathState(-1);
                }*/
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
