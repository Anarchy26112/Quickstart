package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;
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

import com.pedropathing.follower.Follower;

import java.util.List;

@TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    private static final double BLUE_TARGET_X = 72;
    private static final double BLUE_TARGET_Y = -144.0;

    private Follower follower;

    private DriverControlsBlue driverControlsBlue;
    private OperatorControls operatorControls;
    private LimelightTuning limelightTuning;

    private Intake intake;
    private Gate gate;
    private Shooter shooter;
    private Limelight limelight;

    private int loopCount = 0;
    private static final int TELEMETRY_UPDATE_FREQUENCY = 10;

    private static final boolean TUNING_MODE = false;

    // ----- LOOP DEBUG SETTINGS -----
    private static final boolean LOOP_DEBUG = true;
    private static final boolean LOG_SLOW_LOOPS = true;
    private static final double SLOW_LOOP_THRESHOLD_MS = 25.0;
    private static final int PROFILE_WINDOW = 50;
    // -------------------------------

    private Pose restoredAutoPose = null;
    private Pose restoredTeleopPose = null;

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

        driverControlsBlue = new DriverControlsBlue(follower, telemetry, limelight);

        operatorControls = new OperatorControls(
                follower,
                intake,
                shooter,
                telemetry,
                limelight,
                gate,
                BLUE_TARGET_X,
                BLUE_TARGET_Y
        );

        limelightTuning = new LimelightTuning(intake, shooter, telemetry, limelight);

        if (PoseHandoff.hasPose()) {
            restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                restoredTeleopPose = new Pose(
                        restoredAutoPose.getX(),
                        restoredAutoPose.getY(),
                        restoredAutoPose.getHeading()
                );
                follower.setPose(restoredTeleopPose);
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

        // 1) Bulk cache clear
        for (int i = 0; i < hubCount; i++) {
            allHubs.get(i).clearBulkCache();
        }
        profiler.markBulkCache();

        // 2) Driver controls
        if (driverControlsBlue != null) {
            driverControlsBlue.setIntakingActive(operatorControls != null && operatorControls.isIntaking());
            driverControlsBlue.update(gamepad1);
        }
        profiler.markDriver();

        // 3) Operator controls / auto align logic
        if (operatorControls != null && driverControlsBlue != null) {
            if (!TUNING_MODE) {
                operatorControls.update(gamepad2);
            }

            if (operatorControls.shouldEnableAutoAlign()) {
                driverControlsBlue.forceEnableAutoAlign();
                operatorControls.clearEnableAutoAlignRequest();
            }

            if (operatorControls.shouldDisableAutoAlign()) {
                driverControlsBlue.forceDisableAutoAlign();
                operatorControls.clearDisableAutoAlignRequest();
            }

            operatorControls.setAutoAlignEnabled(driverControlsBlue.isAutoAlignEnabled());
        }
        profiler.markOperator();

        // 4) Limelight tuning
        if (TUNING_MODE && limelightTuning != null) {
            limelightTuning.update(gamepad2);
        }
        profiler.markTuning();

        // 5) Shooter update
        if (shooter != null) {
            shooter.update();
        }
        profiler.markShooter();

        // 6) Follower update
        if (follower != null) {
            follower.update();
        }
        profiler.markFollower();

        // 7) Telemetry
        boolean doTelemetry = (loopCount++ % TELEMETRY_UPDATE_FREQUENCY == 0);
        if (doTelemetry) {
            telemetry.addData("Mode", TUNING_MODE ? "TUNING" : "COMPETITION");

            if (driverControlsBlue != null) driverControlsBlue.updateTelemetry();
            if (!TUNING_MODE && operatorControls != null) operatorControls.updateTelemetry();
            if (TUNING_MODE && limelightTuning != null) limelightTuning.updateTelemetry();

            if (LOOP_DEBUG) {
                profiler.addTelemetry(telemetry);
            }

            telemetry.update();
        }
        profiler.markTelemetry();

        profiler.endLoop();

        if (LOOP_DEBUG && LOG_SLOW_LOOPS && profiler.getLastLoopMs() > SLOW_LOOP_THRESHOLD_MS) {
            RobotLog.ii(
                    "LoopProfiler",
                    "SLOW LOOP: total=%.2f ms | bulk=%.2f | driver=%.2f | operator=%.2f | tuning=%.2f | shooter=%.2f | follower=%.2f | telemetry=%.2f",
                    profiler.getLastLoopMs(),
                    profiler.getLastBulkMs(),
                    profiler.getLastDriverMs(),
                    profiler.getLastOperatorMs(),
                    profiler.getLastTuningMs(),
                    profiler.getLastShooterMs(),
                    profiler.getLastFollowerMs(),
                    profiler.getLastTelemetryMs()
            );
        }
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
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
            loopStartNs = System.nanoTime();
            lastMarkNs = loopStartNs;
        }

        void markBulkCache() {
            lastBulkMs = elapsedSinceLastMarkMs();
            if (lastBulkMs > maxBulkMs) maxBulkMs = lastBulkMs;
        }

        void markDriver() {
            lastDriverMs = elapsedSinceLastMarkMs();
            if (lastDriverMs > maxDriverMs) maxDriverMs = lastDriverMs;
        }

        void markOperator() {
            lastOperatorMs = elapsedSinceLastMarkMs();
            if (lastOperatorMs > maxOperatorMs) maxOperatorMs = lastOperatorMs;
        }

        void markTuning() {
            lastTuningMs = elapsedSinceLastMarkMs();
            if (lastTuningMs > maxTuningMs) maxTuningMs = lastTuningMs;
        }

        void markShooter() {
            lastShooterMs = elapsedSinceLastMarkMs();
            if (lastShooterMs > maxShooterMs) maxShooterMs = lastShooterMs;
        }

        void markFollower() {
            lastFollowerMs = elapsedSinceLastMarkMs();
            if (lastFollowerMs > maxFollowerMs) maxFollowerMs = lastFollowerMs;
        }

        void markTelemetry() {
            lastTelemetryMs = elapsedSinceLastMarkMs();
            if (lastTelemetryMs > maxTelemetryMs) maxTelemetryMs = lastTelemetryMs;
        }

        void endLoop() {
            lastLoopMs = (System.nanoTime() - loopStartNs) / 1_000_000.0;
            if (lastLoopMs > maxLoopMs) maxLoopMs = lastLoopMs;

            sumLoopMs += lastLoopMs;
            loopSamples++;

            if (lastLoopMs > SLOW_LOOP_THRESHOLD_MS) {
                slowLoops++;
            }

            if (loopSamples > PROFILE_WINDOW) {
                // rolling-ish window reset
                sumLoopMs = lastLoopMs;
                loopSamples = 1;
                slowLoops = (lastLoopMs > SLOW_LOOP_THRESHOLD_MS) ? 1 : 0;
            }
        }

        private double elapsedSinceLastMarkMs() {
            long now = System.nanoTime();
            double ms = (now - lastMarkNs) / 1_000_000.0;
            lastMarkNs = now;
            return ms;
        }

        void addTelemetry(Telemetry telemetry) {
            double avgLoopMs = loopSamples > 0 ? sumLoopMs / loopSamples : 0.0;
            double hz = avgLoopMs > 0 ? 1000.0 / avgLoopMs : 0.0;

            telemetry.addLine("===== LOOP PROFILER =====");
            telemetry.addData("Last Loop (ms)", "%.2f", lastLoopMs);
            telemetry.addData("Avg Loop (ms)", "%.2f", avgLoopMs);
            telemetry.addData("Max Loop (ms)", "%.2f", maxLoopMs);
            telemetry.addData("Loop Rate (Hz)", "%.1f", hz);
            telemetry.addData("Slow Loops", "%d / %d", slowLoops, loopSamples);

            telemetry.addLine("--- Sections: last / max (ms) ---");
            telemetry.addData("Bulk Cache", "%.2f / %.2f", lastBulkMs, maxBulkMs);
            telemetry.addData("Driver", "%.2f / %.2f", lastDriverMs, maxDriverMs);
            telemetry.addData("Operator", "%.2f / %.2f", lastOperatorMs, maxOperatorMs);
            telemetry.addData("Tuning", "%.2f / %.2f", lastTuningMs, maxTuningMs);
            telemetry.addData("Shooter", "%.2f / %.2f", lastShooterMs, maxShooterMs);
            telemetry.addData("Follower", "%.2f / %.2f", lastFollowerMs, maxFollowerMs);
            telemetry.addData("Telemetry", "%.2f / %.2f", lastTelemetryMs, maxTelemetryMs);
            telemetry.addData("Runtime (s)", "%.1f", runtime.seconds());
        }

        double getLastLoopMs() { return lastLoopMs; }
        double getLastBulkMs() { return lastBulkMs; }
        double getLastDriverMs() { return lastDriverMs; }
        double getLastOperatorMs() { return lastOperatorMs; }
        double getLastTuningMs() { return lastTuningMs; }
        double getLastShooterMs() { return lastShooterMs; }
        double getLastFollowerMs() { return lastFollowerMs; }
        double getLastTelemetryMs() { return lastTelemetryMs; }
    }
}