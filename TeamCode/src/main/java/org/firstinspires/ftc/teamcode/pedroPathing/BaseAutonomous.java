package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;

import java.util.Locale;

@Autonomous(name = "Base Autonomous", group = "Auto")
public class BaseAutonomous extends OpMode {

    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // Telemetry
    private TelemetryManager telemetryM;

    // Subsystems
    private Intake intake;
    private SpinDex spinDex;
    private Shooter shooter;
    private Pusher pusher;

    // State tracking
    private int pathState;

    // Starting pose - MODIFY FOR EACH AUTO
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // TODO: Add more poses here as needed
    // Example: private final Pose scorePose = new Pose(14, 129, Math.toRadians(-45));

    // Path chains
    // TODO: Add path chains here
    // Example: private PathChain scorePreload;

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug("=== AUTONOMOUS INITIALIZATION ===");

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetryM.debug("✓ Follower initialized");

        // Initialize all subsystems
        intake = new Intake(hardwareMap, telemetryM);
        spinDex = new SpinDex(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        pusher = new Pusher(hardwareMap, telemetryM);
        telemetryM.debug("✓ Subsystems initialized");

        // Build paths
        buildPaths();
        telemetryM.debug("✓ Paths built");

        telemetryM.debug("=== INITIALIZATION COMPLETE ===");
        telemetryM.update();
    }

    @Override
    public void init_loop() {
        // Display status while waiting for start
        telemetryM.debug("=== WAITING FOR START ===");
        telemetryM.debug("✓ All systems ready");
        telemetryM.debug("Robot Pose: " + follower.getPose().toString());
        telemetryM.debug("");

        // Display subsystem status
        telemetryM.debug("--- Subsystem Status ---");
        telemetryM.debug("Intake: " + intake.getState());
        telemetryM.debug("Shooter: " + shooter.getState());
        telemetryM.debug("Pusher: " + pusher.getState());
        telemetryM.debug("SpinDex: " + spinDex.getState());

        telemetryM.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        telemetryM.debug("=== AUTONOMOUS STARTED ===");
        telemetryM.update();
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
        telemetryM.debug("=== AUTONOMOUS RUNNING ===");
        telemetryM.debug("Path State: " + pathState);
        telemetryM.debug("Runtime: " + String.format(Locale.US, "%.1f sec", opmodeTimer.getElapsedTimeSeconds()));
        telemetryM.debug("");
        telemetryM.debug("Robot Position:");
        telemetryM.debug("X: " + String.format(Locale.US, "%.2f", follower.getPose().getX()));
        telemetryM.debug("Y: " + String.format(Locale.US, "%.2f", follower.getPose().getY()));
        telemetryM.debug("Heading: " + String.format(Locale.US, "%.2f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("");
        telemetryM.debug("--- Subsystems ---");
        telemetryM.debug("Intake: " + intake.getState());
        telemetryM.debug("Shooter: " + shooter.getState() +
                " | Vel: " + String.format(Locale.US, "%.0f", shooter.getAverageVelocity()));
        telemetryM.debug("Pusher: " + pusher.getState());
        telemetryM.debug("SpinDex: " + spinDex.getState());

        telemetryM.update();
    }

    @Override
    public void stop() {
        // Emergency stop all subsystems
        intake.stop();
        shooter.stop();
        pusher.stop();

        telemetryM.debug("=== AUTONOMOUS STOPPED ===");
        telemetryM.update();
    }

    // ========== PATH BUILDING ==========

    /**
     * Build all paths for the autonomous
     * This is called once during init
     */
    public void buildPaths() {
        // TODO: Build your paths here using follower.pathBuilder()

        // Example path structure:
        /*
        scorePreload = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0, () -> intake.intake())
                .addParametricCallback(0.5, () -> shooter.spin(0.8))
                .build();
        */
    }

    // ========== PATH STATE MACHINE ==========

    /**
     * Main autonomous path update loop
     * This switch runs continuously and controls the robot's actions
     *
     * IMPORTANT PATTERNS:
     * 1. Case 0: Start the first path with follower.followPath(), then immediately setPathState(1)
     * 2. Wait cases: Use if(!follower.isBusy()) to check if path is complete
     * 3. Timed actions: Use pathTimer.getElapsedTimeSeconds() for delays
     * 4. Chain actions: Can have multiple timed checks in same case before advancing
     * 5. Final case: setPathState(-1) to stop the state machine
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start first path and immediately advance to waiting state
                // Example: Score preload
                // follower.followPath(scorePreload, true);
                // setPathState(1);
                break;

            case 1:
                // Wait for path to complete, then do timed actions
                // if (!follower.isBusy()) {
                //     if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                //         // Action 1: Push sample
                //         pusher.push();
                //     }
                //     if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                //         // Action 2: Stop shooter and advance
                //         shooter.stop();
                //         setPathState(2);
                //     }
                // }
                break;

            case 2:
                // Wait for previous actions to settle, then start next path
                // if (!follower.isBusy()) {
                //     // Start intake for pickup
                //     intake.intake();
                //     spinDex.moveToPosition(1);
                //
                //     follower.followPath(grabSample1, true);
                //     setPathState(3);
                // }
                break;

            case 3:
                // Wait at pickup position and grab sample
                // if (!follower.isBusy()) {
                //     if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                //         // Stop intake once sample collected
                //         intake.stop();
                //         setPathState(4);
                //     }
                // }
                break;

            // Add more cases as needed for your autonomous routine
            // ...

            case -1:
            default:
                // Stop state - autonomous complete, do nothing
                break;
        }
    }

    /**
     * Change path state and reset timer
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ========== HELPER METHODS ==========

    /**
     * Get follower for direct access if needed
     */
    protected Follower getFollower() {
        return follower;
    }

}
