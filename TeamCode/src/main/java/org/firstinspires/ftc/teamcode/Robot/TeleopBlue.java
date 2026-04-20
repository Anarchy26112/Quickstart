package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHandoff;

import java.util.List;

@TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    private static final double BLUE_TARGET_X = 72;
    private static final double BLUE_TARGET_Y = -144.0;

    private static final int TELEMETRY_UPDATE_FREQUENCY = 25;
    private static final boolean TUNING_MODE = false;

    private static final boolean LOOP_DEBUG = false;
    private static final boolean LOG_SLOW_LOOPS = false;
    private static final double SLOW_LOOP_THRESHOLD_MS = 25.0;
    private static final int PROFILE_WINDOW = 50;

    private Follower follower;

    private DriverControlsBlue driverControlsBlue;
    private OperatorControls operatorControls;
    private Intake intake;
    private Gate gate;
    private Shooter shooter;
    private GoalAimController aimController;

    private int loopCount = 0;

    private Pose restoredAutoPose = null;

    private List<LynxModule> allHubs;
    private int hubCount;

    private final LoopProfiler profiler = new LoopProfiler();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        hubCount = allHubs.size();

        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake = new Intake(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        aimController = new GoalAimController(follower, telemetry);

        driverControlsBlue = new DriverControlsBlue(follower, telemetry, aimController);

        operatorControls = new OperatorControls(
                intake,
                shooter,
                telemetry,
                gate,
                aimController,
                BLUE_TARGET_X,
                BLUE_TARGET_Y
        );

        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                follower.setPose(restoredAutoPose);
                PoseHandoff.clear();
            }
        }

        telemetry.addData("Status", "Initialized (Shared Follower + Bulk Caching)");
        telemetry.addData("Loop Debug", LOOP_DEBUG ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driverControlsBlue != null) {
            driverControlsBlue.startTeleopDrive();
        }
        profiler.reset();
    }

    @Override
    public void loop() {
        profiler.startLoop();

        clearAllBulkCaches();
        profiler.markBulkCache();

        final long nowNs = System.nanoTime();
        final long nowMs = nowNs / 1_000_000L;

        updateFollower();
        profiler.markFollower();

        Pose pose = follower != null ? follower.getPose() : null;
        Vector vel = follower != null ? follower.getVelocity() : null;

        updateAimController(pose, nowMs, nowNs);
        profiler.markDriver(); // reuse existing slot, or add a separate aim slot if you want

        updateDriverControls(pose, nowMs);
        profiler.markOperator(); // reuse existing slot, or separate if you want

        updateOperatorControls(pose, vel, nowMs, nowNs);
        profiler.markShooter(); // reuse existing slot, or separate if you want

        updateShooter(nowNs);
        profiler.markTuning();

        updateTuningMode();
        profiler.markTelemetry();

        updateTelemetryBlock(nowMs);

        profiler.endLoop();
        maybeLogSlowLoop();
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
        if (shooter != null) {
            shooter.stop();
        }
    }

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

    private void updateAimController(Pose pose, long nowMs, long nowNs) {
        if (aimController == null || pose == null) return;

        aimController.setRobotPose(pose.getX(), pose.getY(), pose.getHeading());
        aimController.update(nowMs, nowNs);
    }

    private void updateDriverControls(Pose pose, long nowMs) {
        if (driverControlsBlue == null) return;

        driverControlsBlue.setIntakingActive(
                operatorControls != null && operatorControls.isIntaking()
        );
        driverControlsBlue.update(gamepad1, pose, nowMs);
    }

    private void updateShooter(long nowNs) {
        if (shooter != null) {
            shooter.update(nowNs);
        }
    }

    private void updateOperatorControls(Pose pose, Vector vel, long nowMs, long nowNs) {
        if (operatorControls == null || driverControlsBlue == null) return;

        operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());

        if (!TUNING_MODE) {
            operatorControls.update(gamepad1, pose, vel, nowMs, nowNs);
        }

        if (operatorControls.shouldEnableAutoAlign()) {
            driverControlsBlue.forceEnableAutoAlign();
            operatorControls.clearEnableAutoAlignRequest();
            operatorControls.setAutoAlignEnabled(true);
        }

        if (operatorControls.shouldDisableAutoAlign()) {
            driverControlsBlue.forceDisableAutoAlign();
            operatorControls.clearDisableAutoAlignRequest();
            operatorControls.setAutoAlignEnabled(false);
        }
    }

    private void updateTuningMode() {
    }

    private void updateTelemetryBlock(long nowMs) {
        boolean doTelemetry = (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0);
        if (!doTelemetry) return;

        telemetry.addData("Mode", TUNING_MODE ? "TUNING" : "COMPETITION");

        if (driverControlsBlue != null) {
            driverControlsBlue.sendTelemetry();
        }

        if (!TUNING_MODE && operatorControls != null) {
            operatorControls.updateTelemetry(nowMs);
        }

        if (LOOP_DEBUG) {
            profiler.addTelemetry(telemetry);
        }

        telemetry.update();
    }

    private void maybeLogSlowLoop() {
        if (LOOP_DEBUG && LOG_SLOW_LOOPS && profiler.getLastLoopMs() > SLOW_LOOP_THRESHOLD_MS) {
            RobotLog.ii(
                    "LoopProfiler",
                    "SLOW LOOP: total=%.2f ms | bulk=%.2f | follower=%.2f | driver=%.2f | shooter=%.2f | operator=%.2f | tuning=%.2f | telemetry=%.2f",
                    profiler.getLastLoopMs(),
                    profiler.getLastBulkMs(),
                    profiler.getLastFollowerMs(),
                    profiler.getLastDriverMs(),
                    profiler.getLastShooterMs(),
                    profiler.getLastOperatorMs(),
                    profiler.getLastTuningMs(),
                    profiler.getLastTelemetryMs()
            );
        }
    }

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

        private double maxBulkMs;
        private double maxDriverMs;
        private double maxOperatorMs;
        private double maxTuningMs;
        private double maxShooterMs;
        private double maxFollowerMs;
        private double maxTelemetryMs;
        private double maxLoopMs;

        private double sumLoopMs;
        private int loopSamples;
        private int slowLoops;

        private final ElapsedTime runtime = new ElapsedTime();

        void reset() {
            loopStartNs = 0;
            lastMarkNs = 0;

            lastBulkMs = 0;
            lastDriverMs = 0;
            lastOperatorMs = 0;
            lastTuningMs = 0;
            lastShooterMs = 0;
            lastFollowerMs = 0;
            lastTelemetryMs = 0;
            lastLoopMs = 0;

            maxBulkMs = 0;
            maxDriverMs = 0;
            maxOperatorMs = 0;
            maxTuningMs = 0;
            maxShooterMs = 0;
            maxFollowerMs = 0;
            maxTelemetryMs = 0;
            maxLoopMs = 0;

            sumLoopMs = 0;
            loopSamples = 0;
            slowLoops = 0;

            runtime.reset();
        }

        void startLoop() {
            long nowNs = System.nanoTime();

            if (loopStartNs != 0) {
                lastLoopMs = (nowNs - loopStartNs) / 1_000_000.0;
                if (lastLoopMs > maxLoopMs) maxLoopMs = lastLoopMs;

                sumLoopMs += lastLoopMs;
                loopSamples++;

                if (lastLoopMs > SLOW_LOOP_THRESHOLD_MS) {
                    slowLoops++;
                }

                if (loopSamples > PROFILE_WINDOW) {
                    sumLoopMs = lastLoopMs;
                    loopSamples = 1;
                    slowLoops = (lastLoopMs > SLOW_LOOP_THRESHOLD_MS) ? 1 : 0;
                }
            }

            loopStartNs = nowNs;
            lastMarkNs = loopStartNs;
        }

        void markBulkCache() {
            lastBulkMs = elapsedSinceLastMarkMs();
            if (lastBulkMs > maxBulkMs) maxBulkMs = lastBulkMs;
        }

        void markFollower() {
            lastFollowerMs = elapsedSinceLastMarkMs();
            if (lastFollowerMs > maxFollowerMs) maxFollowerMs = lastFollowerMs;
        }

        void markDriver() {
            lastDriverMs = elapsedSinceLastMarkMs();
            if (lastDriverMs > maxDriverMs) maxDriverMs = lastDriverMs;
        }

        void markShooter() {
            lastShooterMs = elapsedSinceLastMarkMs();
            if (lastShooterMs > maxShooterMs) maxShooterMs = lastShooterMs;
        }

        void markOperator() {
            lastOperatorMs = elapsedSinceLastMarkMs();
            if (lastOperatorMs > maxOperatorMs) maxOperatorMs = lastOperatorMs;
        }

        void markTuning() {
            lastTuningMs = elapsedSinceLastMarkMs();
            if (lastTuningMs > maxTuningMs) maxTuningMs = lastTuningMs;
        }

        void markTelemetry() {
            lastTelemetryMs = elapsedSinceLastMarkMs();
            if (lastTelemetryMs > maxTelemetryMs) maxTelemetryMs = lastTelemetryMs;
        }

        void endLoop() {
        }

        private double elapsedSinceLastMarkMs() {
            long now = System.nanoTime();
            double ms = (now - lastMarkNs) / 1_000_000.0;
            lastMarkNs = now;
            return ms;
        }

        double getLastBulkMs() { return lastBulkMs; }
        double getLastDriverMs() { return lastDriverMs; }
        double getLastOperatorMs() { return lastOperatorMs; }
        double getLastTuningMs() { return lastTuningMs; }
        double getLastShooterMs() { return lastShooterMs; }
        double getLastFollowerMs() { return lastFollowerMs; }
        double getLastTelemetryMs() { return lastTelemetryMs; }
        double getLastLoopMs() { return lastLoopMs; }

        void addTelemetry(Telemetry telemetry) {
            double avgLoopMs = loopSamples > 0 ? (sumLoopMs / loopSamples) : 0.0;

            telemetry.addData("Loop ms", "%.2f", lastLoopMs);
            telemetry.addData("Loop avg ms", "%.2f", avgLoopMs);
            telemetry.addData("Loop slow", "%d/%d", slowLoops, loopSamples);
            telemetry.addData("Bulk ms", "%.2f", lastBulkMs);
            telemetry.addData("Follower ms", "%.2f", lastFollowerMs);
            telemetry.addData("Driver ms", "%.2f", lastDriverMs);
            telemetry.addData("Operator ms", "%.2f", lastOperatorMs);
            telemetry.addData("Shooter ms", "%.2f", lastShooterMs);
            telemetry.addData("Tuning ms", "%.2f", lastTuningMs);
            telemetry.addData("Telemetry ms", "%.2f", lastTelemetryMs);
            telemetry.addData("Runtime", "%.1f", runtime.seconds());
        }
    }
}