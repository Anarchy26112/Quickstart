package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NANO_TO_MS;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NANO_TO_SEC;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Helpers.Alliance;
import org.firstinspires.ftc.teamcode.Helpers.PoseHandoff;
import org.firstinspires.ftc.teamcode.Robot.Controls.DriverControls;
import org.firstinspires.ftc.teamcode.Robot.Controls.OperatorControls;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "TeleOp Blue")
public class TeleopBlue extends OpMode {

    private static final boolean TUNING_MODE       = true;
    private static final boolean LOOP_DEBUG        = true;
    private static final boolean LOG_SLOW_LOOPS    = false;

    private static final long TELEMETRY_INTERVAL_MS = 250L;
    private static final long VOLTAGE_IDLE_UPDATE_INTERVAL_NS = 250_000_000L; // 250 ms
    private static final long VOLTAGE_AIM_UPDATE_INTERVAL_NS  = 50_000_000L;  // 50 ms

    private static final double SLOW_LOOP_THRESHOLD_MS = 15.0;
    private static final int    PROFILE_WINDOW         = 50;
    private static final double RAD_TO_DEG             = 180.0 / Math.PI;

    private Follower          follower;
    private DriverControls    driverControls;
    private OperatorControls  operatorControls;
    private Shooter           shooter;
    private GoalAimController aimController;

    private LynxModule hub0 = null;
    private LynxModule hub1 = null;
    private LynxModule shooterHub = null;

    private LoopProfiler profiler;

    private long   lastTelemetryMs      = 0L;
    private long   lastLoopNs           = 0L;
    private double cachedVoltageComp    = 1.0;
    private long   lastVoltageUpdateNs  = 0L;

    // This lets us call aimController.forceIdle() only once
    // when auto-align changes from ON to OFF.
    private boolean wasAutoAlignActiveLastLoop = false;

    @Override
    public void init() {
        if (TUNING_MODE) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        // Initialize hubs for manual bulk caching.
        // Control Hub is usually the parent hub.
        // Expansion Hub is usually the non-parent hub.
        final List<LynxModule> hubsList = hardwareMap.getAll(LynxModule.class);
        if (!hubsList.isEmpty()) {
            for (LynxModule hub : hubsList) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

                if (hub.isParent()) {
                    hub0 = hub;      // Usually Control Hub
                } else {
                    hub1 = hub;      // Usually Expansion Hub
                    shooterHub = hub;
                }
            }

            // Fallback: if there is only one hub, use it for shooter voltage.
            if (shooterHub == null) {
                shooterHub = hubsList.get(0);
            }
        }

        final Intake intake = new Intake(hardwareMap, telemetry);
        final Gate   gate   = new Gate(hardwareMap);

        shooter  = new Shooter(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);

        // One initial follower update to initialize pose state.
        follower.update();

        aimController = new GoalAimController(telemetry);

        driverControls   = new DriverControls(follower, telemetry, aimController, Alliance.BLUE);
        operatorControls = new OperatorControls(intake, shooter, telemetry, gate, aimController);

        if (PoseHandoff.hasPose()) {
            final Pose restoredAutoPose = PoseHandoff.get();
            if (restoredAutoPose != null) {
                follower.setPose(restoredAutoPose);
                PoseHandoff.clear();
            }
        }

        if (LOOP_DEBUG) {
            profiler = new LoopProfiler();
        }
    }

    @Override
    public void start() {
        driverControls.startTeleopDrive();

        if (LOOP_DEBUG && profiler != null) {
            profiler.reset();
        }

        lastTelemetryMs     = 0L;
        lastLoopNs          = System.nanoTime();
        lastVoltageUpdateNs = 0L;
        cachedVoltageComp   = 1.0;

        wasAutoAlignActiveLastLoop = false;
    }

    @Override
    public void loop() {
        final long nowNs = System.nanoTime();

        if (LOOP_DEBUG && profiler != null) {
            profiler.startLoop(nowNs);
        }

        // 1. Clear bulk cache ONCE per loop.
        if (hub0 != null) hub0.clearBulkCache();
        if (hub1 != null) hub1.clearBulkCache();

        final long nowMs = nowNs / 1_000_000L;

        // 2. Calculate delta time efficiently.
        double loopDtSec = (nowNs - lastLoopNs) * NANO_TO_SEC;
        lastLoopNs = nowNs;

        if      (loopDtSec < 0.0001) loopDtSec = 0.0001;
        else if (loopDtSec > 0.1)    loopDtSec = 0.1;

        // 3. Process driver inputs and pose requests.
        driverControls.readInputs(gamepad1);
        driverControls.handlePoseRequests();

        // 4. Update odometry and fetch pose ONCE.
        follower.update();

        final Pose pose = follower.getPose();
        if (pose == null) return;

        final double rX       = pose.getX();
        final double rY       = pose.getY();
        final double rHeading = pose.getHeading();

        // 5. Manage subsystem state.
        boolean autoAlignActive = driverControls.isAutoAlignEnabled();

        operatorControls.setAutoAlignEnabled(autoAlignActive);

        // Current code uses gamepad1 for operator controls.
        operatorControls.update(gamepad1, rX, rY, nowMs, nowNs, loopDtSec);

        if (operatorControls.shouldDisableAutoAlign()) {
            autoAlignActive = false;

            driverControls.forceDisableAutoAlign();
            operatorControls.setAutoAlignEnabled(false);
            operatorControls.clearDisableAutoAlignRequest();
        }

        // 6. Voltage compensation.
        // Shooter is always spun up, but voltage is only polled quickly during
        // auto-align or the actual shooting state.
        final boolean forceVoltageRefresh = operatorControls.consumeJustStartedShooting();
        final boolean shootingActive      = operatorControls.isShooting();

        final boolean voltageFastMode =
                autoAlignActive || shootingActive || forceVoltageRefresh;

        final double currentVoltageComp =
                getVoltageComp(nowNs, voltageFastMode, forceVoltageRefresh);

        // 7. Update aim only when active.
        // If auto-align just turned off, reset aim once.
        if (autoAlignActive) {
            aimController.updateActive(rX, rY, rHeading, nowNs, loopDtSec);
        } else if (wasAutoAlignActiveLastLoop) {
            aimController.forceIdle(nowNs);
        }

        wasAutoAlignActiveLastLoop = autoAlignActive;

        // 8. Push final drive state.
        driverControls.applyDrive(autoAlignActive);

        // 9. Update shooter.
        shooter.setRobotY(rY);
        shooter.update(currentVoltageComp, loopDtSec);

        // 10. Throttled telemetry.
        if (TUNING_MODE && nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
            operatorControls.updateTelemetry(nowMs);

            shooter.telemetry();

            telemetry.addData("Odo X",           rX);
            telemetry.addData("Odo Y",           rY);
            telemetry.addData("Odo Heading Deg", rHeading * RAD_TO_DEG);
            telemetry.addData("Voltage Comp",    currentVoltageComp);
            telemetry.addData("Voltage Mode",    voltageFastMode ? "FAST" : "IDLE");

            if (LOOP_DEBUG && profiler != null) {
                profiler.addTelemetry(telemetry);
            }

            telemetry.update();
            lastTelemetryMs = nowMs;
        }

        if (LOOP_DEBUG && profiler != null) {
            profiler.endLoop(System.nanoTime());

            if (LOG_SLOW_LOOPS && profiler.getLastLoopMs() > SLOW_LOOP_THRESHOLD_MS) {
                RobotLog.ii("LoopProfiler", "SLOW LOOP DETECTED!");
            }
        }
    }

    private double getVoltageComp(
            final long nowNs,
            final boolean fastMode,
            final boolean forceRefresh
    ) {
        if (shooterHub == null) return cachedVoltageComp;

        final long intervalNs =
                fastMode ? VOLTAGE_AIM_UPDATE_INTERVAL_NS : VOLTAGE_IDLE_UPDATE_INTERVAL_NS;

        if (!forceRefresh && nowNs - lastVoltageUpdateNs < intervalNs) {
            return cachedVoltageComp;
        }

        lastVoltageUpdateNs = nowNs;

        // Poll only the Expansion Hub because both shooter motors are wired there.
        double voltage = shooterHub.getInputVoltage(VoltageUnit.VOLTS);

        if (voltage <= 0.0 || Double.isNaN(voltage)) {
            return cachedVoltageComp;
        }

        if      (voltage < 8.0)  voltage = 8.0;
        else if (voltage > 14.0) voltage = 14.0;

        final double ratio = NOMINAL_VOLTAGE / voltage;

        double rawComp;
        if (VOLTAGE_COMP_POWER == 1.0) {
            rawComp = ratio;
        } else if (VOLTAGE_COMP_POWER == 0.0) {
            rawComp = 1.0;
        } else {
            rawComp = Math.pow(ratio, VOLTAGE_COMP_POWER);
        }

        if      (rawComp < 0.85) rawComp = 0.85;
        else if (rawComp > 1.45) rawComp = 1.45;

        cachedVoltageComp = rawComp;
        return cachedVoltageComp;
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
    }

    private static class LoopProfiler {
        private long   loopStartNs;
        private double lastLoopMs;
        private double maxLoopMs;
        private double sumLoopMs;
        private int    loopSamples;
        private int    slowLoops;

        void reset() {
            loopStartNs = 0L;
            lastLoopMs  = 0.0;
            maxLoopMs   = 0.0;
            sumLoopMs   = 0.0;
            loopSamples = 0;
            slowLoops   = 0;
        }

        void startLoop(long nowNs) {
            loopStartNs = nowNs;
        }

        void endLoop(long nowNs) {
            if (loopStartNs == 0L) return;

            lastLoopMs = (nowNs - loopStartNs) * NANO_TO_MS;

            if (lastLoopMs > maxLoopMs) {
                maxLoopMs = lastLoopMs;
            }

            sumLoopMs += lastLoopMs;
            loopSamples++;

            if (lastLoopMs > SLOW_LOOP_THRESHOLD_MS) {
                slowLoops++;
            }

            if (loopSamples > PROFILE_WINDOW) {
                sumLoopMs   = lastLoopMs;
                loopSamples = 1;
                slowLoops   = lastLoopMs > SLOW_LOOP_THRESHOLD_MS ? 1 : 0;
            }
        }

        double getLastLoopMs() {
            return lastLoopMs;
        }

        void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
            telemetry.addData("Loop ms",     lastLoopMs);
            telemetry.addData("Loop avg ms", loopSamples > 0 ? sumLoopMs / loopSamples : 0.0);
            telemetry.addData("Loop max ms", maxLoopMs);
            telemetry.addData("Slow loops",  slowLoops);
        }
    }
}