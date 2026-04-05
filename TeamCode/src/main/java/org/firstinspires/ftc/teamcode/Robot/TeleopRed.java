package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "TeleOp Red")
public class TeleopRed extends OpMode {

    private static final double RED_TARGET_X = 72;
    private static final double RED_TARGET_Y = 144.0;

    private static final int TELEMETRY_UPDATE_FREQUENCY = 1;
    private static final boolean TUNING_MODE = false;

    private Follower follower;

    private DriverControlsRed driverControlsRed;
    private OperatorControls operatorControls;
    private LimelightTuning limelightTuning;

    private Intake intake;
    private Gate gate;
    private Shooter shooter;
    private Limelight limelight;

    private int loopCount = 0;

    private Pose restoredAutoPose = null;

    private List<LynxModule> allHubs;
    private int hubCount;

    private final LoopProfiler profiler = new LoopProfiler();

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        hubCount = allHubs.size();

        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        driverControlsRed = new DriverControlsRed(follower, telemetry, limelight);

        operatorControls = new OperatorControls(
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                RED_TARGET_X,
                RED_TARGET_Y
        );

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                follower.setPose(restoredAutoPose);
                PoseHandoff.clear();
            }
        }

        telemetry.addData("Status", "Initialized (RED mirrored to BLUE)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsRed != null) {
            driverControlsRed.startTeleopDrive();
        }
        profiler.reset();
    }

    @Override
    public void loop() {
        profiler.startLoop();

        clearAllBulkCaches();
        profiler.markBulkCache();

        long nowMs = System.nanoTime() / 1_000_000;

        updateFollower();
        profiler.markFollower();

        Pose pose = follower != null ? follower.getPose() : null;
        Vector vel = follower != null ? follower.getVelocity() : null;

        updateDriverControls(pose, nowMs);
        profiler.markDriver();

        updateOperatorControls(pose, vel, nowMs);
        profiler.markOperator();

        updateShooter();
        profiler.markShooter();

        updateTuningMode();
        profiler.markTuning();

        updateTelemetryBlock(pose, nowMs);
        profiler.markTelemetry();

        profiler.endLoop();
    }

    @Override
    public void stop() {
        if (operatorControls != null) operatorControls.stopAll();
        if (shooter != null) shooter.stop();
    }

    // =========================
    // Modular Loop Functions
    // =========================

    private void clearAllBulkCaches() {
        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).clearBulkCache();
        }
    }

    private void updateFollower() {
        if (follower != null) {
            follower.update();
        }
    }

    private void updateDriverControls(Pose pose, long nowMs) {
        if (driverControlsRed == null) return;

        driverControlsRed.update(gamepad1, pose, nowMs);
    }

    private void updateOperatorControls(Pose pose, Vector vel, long nowMs) {
        if (operatorControls == null || driverControlsRed == null) return;

        operatorControls.setAutoAlignEnabled(driverControlsRed.isAutoAlignEnabled());

        if (!TUNING_MODE) {
            operatorControls.update(gamepad2, pose, vel, nowMs);
        }
    }

    private void updateShooter() {
        if (shooter != null) shooter.update();
    }

    private void updateTuningMode() {
        if (TUNING_MODE && limelightTuning != null) {
            limelightTuning.update(gamepad2);
        }
    }

    private void updateTelemetryBlock(Pose pose, long nowMs) {
        if (loopCount++ % TELEMETRY_UPDATE_FREQUENCY != 0) return;

        if (driverControlsRed != null) {
            driverControlsRed.updateTelemetry(pose);
        }

        if (!TUNING_MODE && operatorControls != null) {
            operatorControls.updateTelemetry(nowMs);
        }

        if (TUNING_MODE && limelightTuning != null) {
            limelightTuning.updateTelemetry();
        }

        telemetry.update();
    }

    // =========================
    // Loop Profiler (copied from Blue)
    // =========================

    private static class LoopProfiler {
        private long loopStartNs;
        private long lastMarkNs;

        private double lastBulkMs;
        private double lastDriverMs;
        private double lastOperatorMs;
        private double lastTuningMs;
        private double lastShooterMs;
        private double lastFollowerMs;
        private double lastTelemetryMs;
        private double lastLoopMs;

        void reset() {
            loopStartNs = 0;
            lastMarkNs = 0;
        }

        void startLoop() {
            long now = System.nanoTime();
            loopStartNs = now;
            lastMarkNs = now;
        }

        void markBulkCache() { lastBulkMs = elapsed(); }
        void markFollower() { lastFollowerMs = elapsed(); }
        void markDriver() { lastDriverMs = elapsed(); }
        void markOperator() { lastOperatorMs = elapsed(); }
        void markTuning() { lastTuningMs = elapsed(); }
        void markShooter() { lastShooterMs = elapsed(); }
        void markTelemetry() { lastTelemetryMs = elapsed(); }

        void endLoop() {}

        private double elapsed() {
            long now = System.nanoTime();
            double ms = (now - lastMarkNs) / 1_000_000.0;
            lastMarkNs = now;
            return ms;
        }
    }
}