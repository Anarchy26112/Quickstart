package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pusher;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDex;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.SpinDexHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    // Shared follower
    private Follower follower;

    // Control handlers
    private DriverControlsBlue driverControlsBlue;
    private OperatorControls operatorControls;
    private LimelightTuning limelightTuning;

    // Subsystems
    private Intake intake;
    private SpinDex spin_dex;
    private Shooter shooter;
    private Pusher pusher;
    private ColorSensor colorSensor;
    private Limelight limelight;

    // Performance optimization
    private int loopCount = 0;
    private static final int TELEMETRY_UPDATE_FREQUENCY = 5;

    // Loop timing (nanoTime-based)
    private long lastLoopTimeNs = 0;
    private double lastLoopMs = 0;
    private double avgLoopMs = 0;
    private static final double LOOP_ALPHA = 0.1; // smoothing factor for EMA

    private Pose restoredAutoPose = null;
    private Pose restoredTeleopPose = null;

    // =========================
    // PROFILING TOGGLES
    // =========================
    private static final boolean PROFILE_ENABLED = true;

    // Turn these off one at a time to isolate the culprit
    private static final boolean RUN_DRIVER = true;
    private static final boolean RUN_OPERATOR = true;
    private static final boolean RUN_LIMELIGHT_TUNING = true;
    private static final boolean RUN_FOLLOWER = true;
    private static final boolean RUN_TELEMETRY = true;

    // =========================
    // PROFILING DATA
    // =========================
    private double msDriver = 0;
    private double msBridge = 0;
    private double msOperator = 0;
    private double msLimelightTuning = 0;
    private double msFollower = 0;
    private double msTelemetryBuild = 0;
    private double msTelemetryUpdate = 0;
    private double msWholeLoopMeasured = 0;

    private double maxDriver = 0;
    private double maxBridge = 0;
    private double maxOperator = 0;
    private double maxLimelightTuning = 0;
    private double maxFollower = 0;
    private double maxTelemetryBuild = 0;
    private double maxTelemetryUpdate = 0;
    private double maxWholeLoopMeasured = 0;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // 1) Initialize all subsystems
        intake = new Intake(hardwareMap, telemetry);
        spin_dex = new SpinDex(hardwareMap, telemetry);
        SpinDexHandoff.clear();
        shooter = new Shooter(hardwareMap, telemetry);
        pusher = new Pusher(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        // 2) Create ONE shared follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // 3) Initialize control handlers with shared follower
        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);

        operatorControls = new OperatorControls(
                follower,
                intake,
                spin_dex,
                shooter,
                pusher,
                telemetry,
                colorSensor,
                limelight
        );

        limelightTuning = new LimelightTuning(
                intake, spin_dex, shooter, pusher, telemetry, colorSensor, limelight
        );

        // 4) Pose handoff: Auto -> TeleOp
        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();

            if (restoredAutoPose != null) {
                double teleopX = restoredAutoPose.getY();
                double teleopY = -restoredAutoPose.getX();
                double teleopH = restoredAutoPose.getHeading() - Math.toRadians(90);

                restoredTeleopPose = new Pose(teleopX, teleopY, teleopH);

                // Apply to shared follower
                follower.setPose(restoredTeleopPose);

                PoseHandoff.clear();

                telemetry.addData("Restored Pose (Auto)", "X=%.1f Y=%.1f H=%.1f°",
                        restoredAutoPose.getX(), restoredAutoPose.getY(), Math.toDegrees(restoredAutoPose.getHeading()));
                telemetry.addData("Restored Pose (TeleOp)", "X=%.1f Y=%.1f H=%.1f°",
                        restoredTeleopPose.getX(), restoredTeleopPose.getY(), Math.toDegrees(restoredTeleopPose.getHeading()));
            } else {
                telemetry.addData("Restored Pose", "Handoff present, but pose was null");
            }
        } else {
            telemetry.addData("Restored Pose", "NONE");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lastLoopTimeNs = System.nanoTime();

        pusher.push();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) {
            driverControlsBlue.startTeleopDrive();
        }

        // Re-apply pose after startTeleopDrive(), if needed
        if (restoredTeleopPose != null) {
            follower.setPose(restoredTeleopPose);

            telemetry.addData("Pose Reapplied", "TeleOp pose restored after start()");
            telemetry.addData("TeleOp Pose", "X=%.1f Y=%.1f H=%.1f°",
                    restoredTeleopPose.getX(), restoredTeleopPose.getY(), Math.toDegrees(restoredTeleopPose.getHeading()));
        } else {
            telemetry.addData("Pose Reapplied", "No restored pose");
        }

        telemetry.addData("Status", "Started");
        telemetry.update();

        lastLoopTimeNs = System.nanoTime();
        lastLoopMs = 0;
        avgLoopMs = 0;
        loopCount = 0;

        resetProfilingStats();
    }

    @Override
    public void loop() {
        long loopStartNs = System.nanoTime();

        // FTC scheduler period from previous loop to this loop
        if (lastLoopTimeNs != 0) {
            long deltaNs = loopStartNs - lastLoopTimeNs;
            lastLoopMs = deltaNs / 1_000_000.0;
        } else {
            lastLoopMs = 0;
        }
        lastLoopTimeNs = loopStartNs;

        avgLoopMs = (avgLoopMs == 0)
                ? lastLoopMs
                : (LOOP_ALPHA * lastLoopMs + (1 - LOOP_ALPHA) * avgLoopMs);

        long t0, t1;

        // Reset per-loop timings
        msDriver = 0;
        msBridge = 0;
        msOperator = 0;
        msLimelightTuning = 0;
        msFollower = 0;
        msTelemetryBuild = 0;
        msTelemetryUpdate = 0;

        // -------------------------
        // DRIVER
        // -------------------------
        t0 = System.nanoTime();
        if (RUN_DRIVER && driverControlsBlue != null) {
            driverControlsBlue.update(gamepad1);
        }
        t1 = System.nanoTime();
        msDriver = nsToMs(t1 - t0);
        maxDriver = Math.max(maxDriver, msDriver);

        // -------------------------
        // BRIDGE AUTO-ALIGN STATE
        // -------------------------
        t0 = System.nanoTime();
        if (operatorControls != null && driverControlsBlue != null) {
            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }
        t1 = System.nanoTime();
        msBridge = nsToMs(t1 - t0);
        maxBridge = Math.max(maxBridge, msBridge);

        // -------------------------
        // OPERATOR
        // FIXED: use gamepad2
        // -------------------------
        t0 = System.nanoTime();
        if (RUN_OPERATOR && operatorControls != null) {
            operatorControls.update(gamepad1);
        }
        t1 = System.nanoTime();
        msOperator = nsToMs(t1 - t0);
        maxOperator = Math.max(maxOperator, msOperator);

        // -------------------------
        // LIMELIGHT TUNING
        // -------------------------
        t0 = System.nanoTime();
        if (RUN_LIMELIGHT_TUNING && limelightTuning != null) {
            limelightTuning.update(gamepad2);
        }
        t1 = System.nanoTime();
        msLimelightTuning = nsToMs(t1 - t0);
        maxLimelightTuning = Math.max(maxLimelightTuning, msLimelightTuning);

        // -------------------------
        // FOLLOWER
        // -------------------------
        t0 = System.nanoTime();
        if (RUN_FOLLOWER && follower != null) {
            follower.update();
        }
        t1 = System.nanoTime();
        msFollower = nsToMs(t1 - t0);
        maxFollower = Math.max(maxFollower, msFollower);

        // -------------------------
        // TELEMETRY
        // -------------------------
        if (RUN_TELEMETRY && loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0) {
            t0 = System.nanoTime();

            telemetry.addData("Loop Time (ms)", "%.2f", lastLoopMs);
            telemetry.addData("Avg Loop (ms)", "%.2f", avgLoopMs);
            if (avgLoopMs > 0) telemetry.addData("Loop Rate (Hz)", "%.1f", 1000.0 / avgLoopMs);

            if (PROFILE_ENABLED) {
                telemetry.addLine("=== PROFILING (THIS SAMPLE) ===");
                telemetry.addData("driverControls.update()", "%.3f ms", msDriver);
                telemetry.addData("bridge autoAlign", "%.3f ms", msBridge);
                telemetry.addData("operatorControls.update()", "%.3f ms", msOperator);
                telemetry.addData("limelightTuning.update()", "%.3f ms", msLimelightTuning);
                telemetry.addData("follower.update()", "%.3f ms", msFollower);
                telemetry.addData("telemetry build", "%.3f ms", msTelemetryBuild); // previous loop value until measured below
                telemetry.addData("telemetry.update()", "%.3f ms", msTelemetryUpdate); // previous loop value until measured below

                telemetry.addLine("=== PROFILING (MAX) ===");
                telemetry.addData("max driver", "%.3f ms", maxDriver);
                telemetry.addData("max bridge", "%.3f ms", maxBridge);
                telemetry.addData("max operator", "%.3f ms", maxOperator);
                telemetry.addData("max limelightTuning", "%.3f ms", maxLimelightTuning);
                telemetry.addData("max follower", "%.3f ms", maxFollower);
                telemetry.addData("max telemetry build", "%.3f ms", maxTelemetryBuild);
                telemetry.addData("max telemetry update", "%.3f ms", maxTelemetryUpdate);
                telemetry.addData("max measured loop body", "%.3f ms", maxWholeLoopMeasured);

                telemetry.addLine("=== TOGGLES ===");
                telemetry.addData("RUN_DRIVER", RUN_DRIVER);
                telemetry.addData("RUN_OPERATOR", RUN_OPERATOR);
                telemetry.addData("RUN_LIMELIGHT_TUNING", RUN_LIMELIGHT_TUNING);
                telemetry.addData("RUN_FOLLOWER", RUN_FOLLOWER);
                telemetry.addData("RUN_TELEMETRY", RUN_TELEMETRY);
            }

            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry();
            if (operatorControls != null) operatorControls.updateTelemetry();
            if (limelightTuning != null) limelightTuning.updateTelemetry();

            t1 = System.nanoTime();
            msTelemetryBuild = nsToMs(t1 - t0);
            maxTelemetryBuild = Math.max(maxTelemetryBuild, msTelemetryBuild);

            t0 = System.nanoTime();
            telemetry.update();
            t1 = System.nanoTime();
            msTelemetryUpdate = nsToMs(t1 - t0);
            maxTelemetryUpdate = Math.max(maxTelemetryUpdate, msTelemetryUpdate);
        }

        long loopEndNs = System.nanoTime();
        msWholeLoopMeasured = nsToMs(loopEndNs - loopStartNs);
        maxWholeLoopMeasured = Math.max(maxWholeLoopMeasured, msWholeLoopMeasured);
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private double nsToMs(long ns) {
        return ns / 1_000_000.0;
    }

    private void resetProfilingStats() {
        msDriver = 0;
        msBridge = 0;
        msOperator = 0;
        msLimelightTuning = 0;
        msFollower = 0;
        msTelemetryBuild = 0;
        msTelemetryUpdate = 0;
        msWholeLoopMeasured = 0;

        maxDriver = 0;
        maxBridge = 0;
        maxOperator = 0;
        maxLimelightTuning = 0;
        maxFollower = 0;
        maxTelemetryBuild = 0;
        maxTelemetryUpdate = 0;
        maxWholeLoopMeasured = 0;
    }
}