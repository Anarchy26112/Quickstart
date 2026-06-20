package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NANO_TO_MS;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NANO_TO_SEC;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.VOLTAGE_COMP_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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

@TeleOp(name = "TeleOp Red")
public class TeleopRed extends OpMode {

    private static final boolean TUNING_MODE       = true;
    private static final boolean LOOP_DEBUG        = true;
    private static final boolean LOG_SLOW_LOOPS    = false;

    private static final long TELEMETRY_INTERVAL_MS = 250L;
    private static final long VOLTAGE_IDLE_UPDATE_INTERVAL_NS = 250_000_000L; // 250 ms
    private static final long VOLTAGE_AIM_UPDATE_INTERVAL_NS  = 30_000_000L;  // 50 ms

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

    // For your robot:
    // driveHub    = Control Hub, because drive motors are wired there.
    // shooterHub  = Expansion Hub, because shooter motors are wired there.
    private LynxModule driveHub = null;
    private LynxModule shooterHub = null;

    private LoopProfiler profiler;

    private long lastTelemetryMs = 0L;
    private long lastLoopNs      = 0L;

    // Separate voltage-comp caches.
    // Do NOT share one cache between the Control Hub and Expansion Hub.
    private double cachedAimVoltageComp     = 1.0;
    private double cachedShooterVoltageComp = 1.0;

    private long lastAimVoltageUpdateNs     = 0L;
    private long lastShooterVoltageUpdateNs = 0L;

    // This lets us call aimController.forceIdle() only once
    // when auto-align changes from ON to OFF.
    private boolean wasAutoAlignActiveLastLoop = false;

    @Override
    public void init() {
        if (TUNING_MODE) {
            telemetry = new MultipleTelemetry(
                    telemetry,
                    FtcDashboard.getInstance().getTelemetry()
            );
        }

        // Initialize hubs for manual bulk caching.
        //
        // Control Hub is usually the parent hub.
        // Expansion Hub is usually the non-parent hub.
        //
        // Your robot:
        // - drive motors are on the Control Hub
        // - shooter motors are on the Expansion Hub
        final List<LynxModule> hubsList = hardwareMap.getAll(LynxModule.class);
        if (!hubsList.isEmpty()) {
            for (LynxModule hub : hubsList) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

                if (hub.isParent()) {
                    hub0 = hub;        // Usually Control Hub
                    driveHub = hub;    // Drive motors are wired here
                } else {
                    hub1 = hub;          // Usually Expansion Hub
                    shooterHub = hub;    // Shooter motors are wired here
                }
            }

            // Fallbacks for unusual hardware configurations or one-hub testing.
            if (driveHub == null) {
                driveHub = hubsList.get(0);
            }

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

        driverControls   = new DriverControls(follower, telemetry, aimController, Alliance.RED);
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

        lastTelemetryMs = 0L;
        lastLoopNs      = System.nanoTime();

        cachedAimVoltageComp     = 1.0;
        cachedShooterVoltageComp = 1.0;

        lastAimVoltageUpdateNs     = 0L;
        lastShooterVoltageUpdateNs = 0L;

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

        /*
         * Pedro velocity handling:
         *
         * follower.getVelocity() returns a Vector for TRANSLATIONAL velocity.
         * Use it only for field X/Y movement speed.
         *
         * Important:
         *     velocity.getHeading() is the direction of the velocity vector.
         *     It is NOT the robot's yaw/angular velocity.
         *
         * For robot yaw/angular velocity, use:
         *     follower.getAngularVelocity()
         */
        final Vector velocity = follower.getVelocity();

        final double rVx;
        final double rVy;

        if (velocity != null) {
            rVx = velocity.getXComponent();
            rVy = velocity.getYComponent();
        } else {
            rVx = Double.NaN;
            rVy = Double.NaN;
        }

        // Pedro robot yaw/angular velocity in radians/sec.
        final double rHeadingVel = follower.getAngularVelocity();

        // 5. Manage subsystem state.
        boolean autoAlignActive =
                driverControls.isAutoAlignEnabled()
                        && !driverControls.isPathOverrideActive();

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
        //
        // Aim / drive voltage compensation:
        //     Read from Control Hub, because drive motors are wired there.
        //
        // Shooter voltage compensation:
        //     Read from Expansion Hub, because shooter motors are wired there.
        final boolean forceVoltageRefresh = operatorControls.consumeJustStartedShooting();
        final boolean shootingActive      = operatorControls.isShooting();

        final boolean voltageFastMode =
                autoAlignActive || shootingActive || forceVoltageRefresh;

        final double currentAimVoltageComp =
                getAimVoltageComp(nowNs, voltageFastMode, forceVoltageRefresh);

        final double currentShooterVoltageComp =
                getShooterVoltageComp(nowNs, voltageFastMode, forceVoltageRefresh);

        // 7. Update aim only when active.
        // If auto-align just turned off, reset aim once.
        if (autoAlignActive) {
            aimController.updateActive(
                    rX,
                    rY,
                    rHeading,
                    nowNs,
                    loopDtSec,
                    rVx,
                    rVy,
                    rHeadingVel,
                    currentAimVoltageComp
            );
        } else if (wasAutoAlignActiveLastLoop) {
            aimController.forceIdle(nowNs);
        }

        wasAutoAlignActiveLastLoop = autoAlignActive;

        // 8. Push final drive state.
        driverControls.applyDrive(autoAlignActive, pose);

        // 9. Update shooter.
        shooter.setRobotY(rY);
        shooter.update(currentShooterVoltageComp, loopDtSec);

        // 10. Throttled telemetry.
        if (TUNING_MODE && nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
            operatorControls.updateTelemetry(nowMs);

            shooter.telemetry();

            telemetry.addData("Odo X",           rX);
            telemetry.addData("Odo Y",           rY);
            telemetry.addData("Odo Heading Deg", rHeading * RAD_TO_DEG);

            telemetry.addData("Pedro Vel X", rVx);
            telemetry.addData("Pedro Vel Y", rVy);
            telemetry.addData("Pedro Angular Vel Deg/Sec", rHeadingVel * RAD_TO_DEG);

            telemetry.addData("Aim Voltage Comp", currentAimVoltageComp);
            telemetry.addData("Shooter Voltage Comp", currentShooterVoltageComp);

            telemetry.addData("Aim Voltage Source", driveHub == hub0 ? "Control Hub" : "Fallback");
            telemetry.addData("Shooter Voltage Source", shooterHub == hub1 ? "Expansion Hub" : "Fallback");

            telemetry.addData("Voltage Mode", voltageFastMode ? "FAST" : "IDLE");

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

    private double getAimVoltageComp(
            final long nowNs,
            final boolean fastMode,
            final boolean forceRefresh
    ) {
        if (driveHub == null) return cachedAimVoltageComp;

        final long intervalNs =
                fastMode ? VOLTAGE_AIM_UPDATE_INTERVAL_NS : VOLTAGE_IDLE_UPDATE_INTERVAL_NS;

        if (!forceRefresh && nowNs - lastAimVoltageUpdateNs < intervalNs) {
            return cachedAimVoltageComp;
        }

        lastAimVoltageUpdateNs = nowNs;

        cachedAimVoltageComp =
                calculateVoltageCompFromHub(driveHub, cachedAimVoltageComp);

        return cachedAimVoltageComp;
    }

    private double getShooterVoltageComp(
            final long nowNs,
            final boolean fastMode,
            final boolean forceRefresh
    ) {
        if (shooterHub == null) return cachedShooterVoltageComp;

        final long intervalNs =
                fastMode ? VOLTAGE_AIM_UPDATE_INTERVAL_NS : VOLTAGE_IDLE_UPDATE_INTERVAL_NS;

        if (!forceRefresh && nowNs - lastShooterVoltageUpdateNs < intervalNs) {
            return cachedShooterVoltageComp;
        }

        lastShooterVoltageUpdateNs = nowNs;

        cachedShooterVoltageComp =
                calculateVoltageCompFromHub(shooterHub, cachedShooterVoltageComp);

        return cachedShooterVoltageComp;
    }

    private double calculateVoltageCompFromHub(
            final LynxModule voltageHub,
            final double fallbackComp
    ) {
        if (voltageHub == null) {
            return fallbackComp;
        }

        double voltage = voltageHub.getInputVoltage(VoltageUnit.VOLTS);

        if (voltage <= 0.0 || Double.isNaN(voltage)) {
            return fallbackComp;
        }

        // Keep bad readings from creating a huge command spike.
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

        return rawComp;
    }

    @Override
    public void stop() {
        if (operatorControls != null) {
            operatorControls.stopAll();
        }
        if (driverControls != null) {
            driverControls.cancelDriveOverrides();
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