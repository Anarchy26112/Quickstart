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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

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

    private static final boolean TUNING_MODE = true;
    private static final boolean LOOP_DEBUG = false;
    private static final boolean LOG_SLOW_LOOPS = false;

    private static final long TELEMETRY_INTERVAL_MS =
            250L;

    private static final long VOLTAGE_CACHE_INTERVAL_MS =
            50L;

    private static final double SLOW_LOOP_THRESHOLD_MS =
            15.0;

    private static final int PROFILE_WINDOW =
            50;

    private static final double RAD_TO_DEG =
            180.0 / Math.PI;

    private Follower follower;
    private DriverControls driverControls;
    private OperatorControls operatorControls;

    /*
     * Intake is now a class field so the loop can send it
     * the current cached voltage-compensation multiplier.
     */
    private Intake intake;

    private Shooter shooter;
    private GoalAimController aimController;

    private LynxModule hub0 = null;
    private LynxModule hub1 = null;

    private LoopProfiler profiler;

    private long lastTelemetryMs = 0L;
    private long lastLoopNs = 0L;
    private long lastVoltageReadMs = -1L;

    /*
     * Standard timed battery-voltage cache.
     */
    private double cachedBatteryVoltageComp = 1.0;
    private double cachedBatteryVoltageVolts = Double.NaN;

    /*
     * Used to call aimController.forceIdle() only once
     * when auto-align changes from active to inactive.
     */
    private boolean wasAutoAlignActiveLastLoop = false;

    @Override
    public void init() {
        if (TUNING_MODE) {
            telemetry =
                    new MultipleTelemetry(
                            telemetry,
                            FtcDashboard
                                    .getInstance()
                                    .getTelemetry()
                    );
        }

        /*
         * Initialize hubs for manual bulk caching.
         */
        final List<LynxModule> hubsList =
                hardwareMap.getAll(
                        LynxModule.class
                );

        if (!hubsList.isEmpty()) {
            for (LynxModule hub : hubsList) {
                hub.setBulkCachingMode(
                        LynxModule.BulkCachingMode.MANUAL
                );

                if (hub.isParent()) {
                    hub0 = hub;
                } else {
                    hub1 = hub;
                }
            }

            if (hub0 == null) {
                hub0 = hubsList.get(0);
            }
        }

        intake =
                new Intake(
                        hardwareMap,
                        telemetry
                );

        final Gate gate =
                new Gate(
                        hardwareMap
                );

        shooter =
                new Shooter(
                        hardwareMap,
                        telemetry
                );

        follower =
                Constants.createFollower(
                        hardwareMap
                );

        /*
         * One initial follower update initializes
         * the pose and velocity state.
         */
        follower.update();

        aimController =
                new GoalAimController(
                        telemetry
                );

        driverControls =
                new DriverControls(
                        follower,
                        telemetry,
                        aimController,
                        Alliance.BLUE
                );

        operatorControls =
                new OperatorControls(
                        intake,
                        shooter,
                        telemetry,
                        gate,
                        aimController
                );

        if (PoseHandoff.hasPose()) {
            final Pose restoredAutoPose =
                    PoseHandoff.get();

            if (restoredAutoPose != null) {
                follower.setPose(
                        restoredAutoPose
                );

                PoseHandoff.clear();
            }
        }

        if (LOOP_DEBUG) {
            profiler =
                    new LoopProfiler();
        }
    }

    @Override
    public void start() {
        driverControls.startTeleopDrive();

        if (LOOP_DEBUG
                && profiler != null) {

            profiler.reset();
        }

        lastTelemetryMs = 0L;
        lastLoopNs = System.nanoTime();

        cachedBatteryVoltageComp = 1.0;
        cachedBatteryVoltageVolts = Double.NaN;
        lastVoltageReadMs = -1L;

        if (intake != null) {
            intake.setVoltageCompensation(
                    1.0
            );
        }

        wasAutoAlignActiveLastLoop = false;
    }

    @Override
    public void loop() {
        final long nowNs =
                System.nanoTime();

        if (LOOP_DEBUG
                && profiler != null) {

            profiler.startLoop(
                    nowNs
            );
        }

        /*
         * 1. Clear each hub bulk cache once per loop.
         */
        if (hub0 != null) {
            hub0.clearBulkCache();
        }

        if (hub1 != null) {
            hub1.clearBulkCache();
        }

        final long nowMs =
                nowNs / 1_000_000L;

        /*
         * 2. Calculate loop delta time.
         */
        double loopDtSec =
                (nowNs - lastLoopNs)
                        * NANO_TO_SEC;

        lastLoopNs = nowNs;

        if (loopDtSec < 0.0001) {
            loopDtSec = 0.0001;
        } else if (loopDtSec > 0.1) {
            loopDtSec = 0.1;
        }

        /*
         * 3. Process driver input and pose requests.
         */
        driverControls.readInputs(
                gamepad1
        );

        driverControls.handlePoseRequests();

        /*
         * 4. Update localization once, then retrieve
         * pose and velocity from the same update cycle.
         */
        follower.update();

        final Pose pose =
                follower.getPose();

        if (pose == null) {
            return;
        }

        final double rX =
                pose.getX();

        final double rY =
                pose.getY();

        final double rHeading =
                pose.getHeading();

        /*
         * PedroPathing returns field-centric localization
         * velocity as a Vector.
         */
        final Vector robotVelocity =
                follower.getVelocity();

        final double rVx;
        final double rVy;

        if (robotVelocity != null) {
            rVx =
                    robotVelocity.getXComponent();

            rVy =
                    robotVelocity.getYComponent();
        } else {
            rVx = 0.0;
            rVy = 0.0;
        }

        final double rHeadingVel =
                follower.getAngularVelocity();

        /*
         * 5. Read battery voltage only when the timed
         * cache expires.
         */
        final double currentVoltageComp =
                getCachedBatteryVoltageComp(
                        nowMs
                );

        /*
         * Apply the same cached voltage multiplier to the
         * intake and transfer motors.
         *
         * This happens before OperatorControls issues this
         * loop's motor commands.
         */
        intake.setVoltageCompensation(
                currentVoltageComp
        );

        /*
         * 6. Manage subsystem state.
         */
        boolean autoAlignActive =
                driverControls.isAutoAlignEnabled()
                        && !driverControls
                        .isPathOverrideActive();

        operatorControls.setAutoAlignEnabled(
                autoAlignActive
        );

        /*
         * The current robot code uses gamepad1 for
         * operator controls.
         *
         * Pass the field-centric X and Y velocities
         * into OperatorControls for moving-shot
         * compensation.
         */
        operatorControls.update(
                gamepad1,
                rX,
                rY,
                rVx,
                rVy,
                nowMs,
                nowNs,
                loopDtSec
        );

        if (operatorControls.shouldDisableAutoAlign()) {
            autoAlignActive = false;

            driverControls.forceDisableAutoAlign();

            operatorControls.setAutoAlignEnabled(
                    false
            );

            operatorControls
                    .clearDisableAutoAlignRequest();
        }

        /*
         * 7. Update active aim.
         *
         * The same measured robot velocity is passed
         * to both the aim controller and shooter
         * compensation logic.
         */
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
                    currentVoltageComp
            );
        } else if (wasAutoAlignActiveLastLoop) {
            aimController.forceIdle(
                    nowNs
            );
        }

        wasAutoAlignActiveLastLoop =
                autoAlignActive;

        /*
         * 8. Push the final drive state.
         */
        driverControls.applyDrive(
                autoAlignActive,
                pose
        );

        /*
         * 9. Update shooter control.
         */
        shooter.setRobotY(
                rY
        );

        shooter.update(
                currentVoltageComp,
                loopDtSec
        );

        /*
         * 10. Throttled telemetry.
         */
        if (TUNING_MODE
                && nowMs - lastTelemetryMs
                >= TELEMETRY_INTERVAL_MS) {

            operatorControls.updateTelemetry(
                    nowMs
            );

            intake.telemetry();
            shooter.telemetry();

            telemetry.addData(
                    "Odo X",
                    rX
            );

            telemetry.addData(
                    "Odo Y",
                    rY
            );

            telemetry.addData(
                    "Odo Heading Deg",
                    rHeading * RAD_TO_DEG
            );

            telemetry.addData(
                    "Pedro Vel X",
                    rVx
            );

            telemetry.addData(
                    "Pedro Vel Y",
                    rVy
            );

            telemetry.addData(
                    "Pedro Speed",
                    robotVelocity != null
                            ? robotVelocity.getMagnitude()
                            : 0.0
            );

            telemetry.addData(
                    "Pedro Angular Vel Deg/Sec",
                    rHeadingVel * RAD_TO_DEG
            );

            telemetry.addData(
                    "Battery Voltage V",
                    "%.2f",
                    cachedBatteryVoltageVolts
            );

            telemetry.addData(
                    "Battery Voltage Comp",
                    currentVoltageComp
            );

            telemetry.addData(
                    "Voltage Cache ms",
                    VOLTAGE_CACHE_INTERVAL_MS
            );

            if (LOOP_DEBUG
                    && profiler != null) {

                profiler.addTelemetry(
                        telemetry
                );
            }

            telemetry.update();

            lastTelemetryMs = nowMs;
        }

        if (LOOP_DEBUG
                && profiler != null) {

            profiler.endLoop(
                    System.nanoTime()
            );

            if (LOG_SLOW_LOOPS
                    && profiler.getLastLoopMs()
                    > SLOW_LOOP_THRESHOLD_MS) {

                RobotLog.ii(
                        "LoopProfiler",
                        "SLOW LOOP DETECTED!"
                );
            }
        }
    }

    private double getCachedBatteryVoltageComp(
            final long nowMs
    ) {
        final boolean cacheExpired =
                lastVoltageReadMs < 0L
                        || nowMs - lastVoltageReadMs
                        >= VOLTAGE_CACHE_INTERVAL_MS;

        if (!cacheExpired) {
            return cachedBatteryVoltageComp;
        }

        lastVoltageReadMs = nowMs;

        final double measuredVoltage =
                readBatteryVoltageVolts();

        if (Double.isFinite(measuredVoltage)
                && measuredVoltage > 0.0) {

            cachedBatteryVoltageVolts =
                    measuredVoltage;

            cachedBatteryVoltageComp =
                    calculateVoltageCompFromVoltage(
                            measuredVoltage,
                            cachedBatteryVoltageComp
                    );
        }

        return cachedBatteryVoltageComp;
    }

    /**
     * Uses the lowest valid battery-voltage value
     * reported by the configured FTC voltage sensors.
     */
    private double readBatteryVoltageVolts() {
        double batteryVoltage =
                Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor
                : hardwareMap.voltageSensor) {

            final double voltage =
                    sensor.getVoltage();

            if (Double.isFinite(voltage)
                    && voltage > 0.0) {

                batteryVoltage =
                        Math.min(
                                batteryVoltage,
                                voltage
                        );
            }
        }

        return Double.isFinite(batteryVoltage)
                ? batteryVoltage
                : Double.NaN;
    }

    private double calculateVoltageCompFromVoltage(
            double voltage,
            final double fallbackComp
    ) {
        if (!Double.isFinite(voltage)
                || voltage <= 0.0) {

            return fallbackComp;
        }

        /*
         * Prevent a bad sensor reading from creating
         * a large command spike.
         */
        if (voltage < 8.0) {
            voltage = 8.0;
        } else if (voltage > 14.0) {
            voltage = 14.0;
        }

        final double ratio =
                NOMINAL_VOLTAGE / voltage;

        final double rawComp;

        if (VOLTAGE_COMP_POWER == 1.0) {
            rawComp = ratio;
        } else if (VOLTAGE_COMP_POWER == 0.0) {
            rawComp = 1.0;
        } else {
            rawComp =
                    Math.pow(
                            ratio,
                            VOLTAGE_COMP_POWER
                    );
        }

        if (rawComp < 0.85) {
            return 0.85;
        }

        if (rawComp > 1.45) {
            return 1.45;
        }

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

        private long loopStartNs;

        private double lastLoopMs;
        private double maxLoopMs;
        private double sumLoopMs;

        private int loopSamples;
        private int slowLoops;

        void reset() {
            loopStartNs = 0L;

            lastLoopMs = 0.0;
            maxLoopMs = 0.0;
            sumLoopMs = 0.0;

            loopSamples = 0;
            slowLoops = 0;
        }

        void startLoop(long nowNs) {
            loopStartNs = nowNs;
        }

        void endLoop(long nowNs) {
            if (loopStartNs == 0L) {
                return;
            }

            lastLoopMs =
                    (nowNs - loopStartNs)
                            * NANO_TO_MS;

            if (lastLoopMs > maxLoopMs) {
                maxLoopMs = lastLoopMs;
            }

            sumLoopMs += lastLoopMs;
            loopSamples++;

            if (lastLoopMs
                    > SLOW_LOOP_THRESHOLD_MS) {

                slowLoops++;
            }

            /*
             * Keep the moving profiling window bounded.
             */
            if (loopSamples > PROFILE_WINDOW) {
                sumLoopMs = lastLoopMs;
                loopSamples = 1;

                slowLoops =
                        lastLoopMs
                                > SLOW_LOOP_THRESHOLD_MS
                                ? 1
                                : 0;
            }
        }

        double getLastLoopMs() {
            return lastLoopMs;
        }

        void addTelemetry(
                org.firstinspires.ftc.robotcore.external.Telemetry telemetry
        ) {
            telemetry.addData(
                    "Loop ms",
                    lastLoopMs
            );

            telemetry.addData(
                    "Loop avg ms",
                    loopSamples > 0
                            ? sumLoopMs / loopSamples
                            : 0.0
            );

            telemetry.addData(
                    "Loop max ms",
                    maxLoopMs
            );

            telemetry.addData(
                    "Slow loops",
                    slowLoops
            );
        }
    }
}