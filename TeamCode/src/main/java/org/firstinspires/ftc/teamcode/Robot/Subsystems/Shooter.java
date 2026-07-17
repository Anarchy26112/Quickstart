package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

/**
 * Dual-flywheel shooter using a custom feedforward + proportional controller.
 *
 * Features:
 * 1. Voltage compensation using a pre-calculated compensation multiplier.
 * 2. Independent control loops for the left and right flywheels.
 * 3. Continuous readiness and settling checks for both flywheels.
 * 4. Requested-power and saturation telemetry for easier tuning.
 */
public class Shooter {

    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final Telemetry telemetry;

    private static final double STOP_VELOCITY = 0.0;
    private static final double MIN_ACTIVE_VELOCITY = 30.0;

    /*
     * These limits should match the limits used by TeleopBlue when it
     * calculates the battery-voltage compensation multiplier.
     */
    private static final double MIN_VOLTAGE_COMP = 0.85;
    private static final double MAX_VOLTAGE_COMP = 1.45;

    private double targetVelocity = STOP_VELOCITY;

    private double currentRVel = STOP_VELOCITY;
    private double currentLVel = STOP_VELOCITY;

    /*
     * Unclipped controller outputs. These are useful for detecting when
     * the controller is asking for more power than the motors can receive.
     */
    private double requestedRightPower = 0.0;
    private double requestedLeftPower = 0.0;

    private double appliedRightPower = 0.0;
    private double appliedLeftPower = 0.0;

    private double lastVoltageCompMultiplier = 1.0;

    private boolean shooterActive = false;
    private boolean readyToShoot = false;
    private boolean controllerSaturated = false;

    /*
     * Currently retained for compatibility with TeleopBlue and possible
     * future zone-specific shooter behavior.
     */
    private boolean isFarZone = true;

    private long readyWindowStartNs = -1L;

    public Shooter(
            HardwareMap hardwareMap,
            Telemetry telemetry
    ) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(
                DcMotorEx.class,
                HW_RIGHT_SHOOTER
        );

        leftShooter = hardwareMap.get(
                DcMotorEx.class,
                HW_LEFT_SHOOTER
        );

        configureMotor(
                rightShooter,
                DcMotor.Direction.FORWARD
        );

        configureMotor(
                leftShooter,
                DcMotor.Direction.REVERSE
        );
    }

    private void configureMotor(
            DcMotorEx motor,
            DcMotor.Direction direction
    ) {
        motor.setDirection(direction);

        /*
         * FLOAT lets the flywheel coast instead of braking aggressively
         * whenever motor power is reduced to zero.
         */
        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        motor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        /*
         * The encoder is still readable in RUN_WITHOUT_ENCODER.
         * This mode allows the custom controller to command raw motor power.
         */
        motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        motor.setPower(0.0);
    }

    /**
     * Records which field zone the robot is in.
     *
     * The zone is currently informational and does not alter controller
     * behavior. The method remains so existing TeleOp code still compiles.
     */
    public void setRobotY(double robotY) {
        if (!Double.isFinite(robotY)) {
            return;
        }

        isFarZone =
                robotY < AIM_FAR_ZONE_Y_THRESHOLD;
    }

    /**
     * Sets the target flywheel velocity.
     *
     * @param velocity target encoder velocity in the same units returned
     *                 by DcMotorEx.getVelocity()
     */
    public void setVelocity(double velocity) {
        /*
         * Reject NaN or infinity rather than allowing invalid values to
         * reach the motor controller.
         */
        final double newTarget;

        if (Double.isFinite(velocity)) {
            newTarget = Math.max(
                    STOP_VELOCITY,
                    velocity
            );
        } else {
            newTarget = STOP_VELOCITY;
        }

        final double targetChange =
                Math.abs(newTarget - targetVelocity);

        if (targetChange > SHOOTER_READY_RESET_EPSILON) {
            resetReadyState();
        }

        targetVelocity = newTarget;

        if (targetVelocity < MIN_ACTIVE_VELOCITY) {
            stop();
            return;
        }

        shooterActive = true;
    }

    /**
     * Updates the shooter using a voltage-compensation multiplier.
     *
     * Example multiplier values:
     * 0.95 = reduce feedforward slightly
     * 1.00 = no compensation
     * 1.08 = increase feedforward by 8%
     *
     * This argument is not raw battery voltage.
     */
    public void update(double voltageCompMultiplier) {
        update(
                voltageCompMultiplier,
                0.0
        );
    }

    /**
     * Runs the custom feedforward + proportional control loop.
     *
     * @param voltageCompMultiplier pre-calculated battery compensation
     *                              multiplier from TeleopBlue
     * @param dtSec                  loop delta time in seconds; retained for
     *                               future acceleration or integral control
     */
    public void update(
            double voltageCompMultiplier,
            double dtSec
    ) {
        /*
         * 1. Read and sanitize current motor velocities.
         */
        currentRVel = sanitizeVelocity(
                rightShooter.getVelocity()
        );

        currentLVel = sanitizeVelocity(
                leftShooter.getVelocity()
        );

        /*
         * 2. Stop immediately when the shooter is inactive.
         */
        if (!shooterActive
                || targetVelocity < MIN_ACTIVE_VELOCITY) {

            stop();
            return;
        }

        /*
         * 3. Validate the already-calculated voltage-compensation
         * multiplier received from TeleopBlue.
         */
        lastVoltageCompMultiplier =
                sanitizeVoltageCompMultiplier(
                        voltageCompMultiplier
                );

        /*
         * 4. Calculate base feedforward power.
         *
         * kV predicts the power needed to maintain the requested velocity.
         * kS helps overcome static friction and drivetrain resistance.
         */
        final double baseFfPower =
                SHOOTER_kV * targetVelocity
                        + SHOOTER_kS;

        /*
         * 5. Apply the compensation multiplier directly.
         *
         * Do not calculate NOMINAL_VOLTAGE / value here because the value
         * received from TeleopBlue is already that compensation ratio.
         */
        final double compensatedFfPower =
                baseFfPower
                        * lastVoltageCompMultiplier;

        /*
         * 6. Calculate independent flywheel errors.
         */
        final double rightError =
                targetVelocity - currentRVel;

        final double leftError =
                targetVelocity - currentLVel;

        /*
         * 7. Calculate the independent FF + P controller outputs.
         */
        requestedRightPower =
                (baseFfPower + SHOOTER_kP * rightError)
                        * lastVoltageCompMultiplier;

        requestedLeftPower =
                (baseFfPower + SHOOTER_kP * leftError)
                        * lastVoltageCompMultiplier;

        /*
         * 8. Detect saturation before clipping.
         */
        controllerSaturated =
                requestedRightPower > 1.0
                        || requestedRightPower < 0.0
                        || requestedLeftPower > 1.0
                        || requestedLeftPower < 0.0;

        /*
         * 9. Clip the outputs to the permitted shooter power range.
         *
         * Negative power is intentionally prohibited so an overspeeding
         * flywheel coasts down instead of reversing or braking aggressively.
         */
        appliedRightPower =
                clipPower(requestedRightPower);

        appliedLeftPower =
                clipPower(requestedLeftPower);

        /*
         * 10. Send the final power commands to the motors.
         */
        rightShooter.setPower(
                appliedRightPower
        );

        leftShooter.setPower(
                appliedLeftPower
        );

        /*
         * 11. Update shooter readiness.
         */
        updateReadyToShoot(
                System.nanoTime(),
                rightError,
                leftError
        );
    }

    private void updateReadyToShoot(
            long nowNs,
            double rightError,
            double leftError
    ) {
        final boolean rightWithinTolerance =
                Math.abs(rightError)
                        <= SHOOTER_READY_TOLERANCE;

        final boolean leftWithinTolerance =
                Math.abs(leftError)
                        <= SHOOTER_READY_TOLERANCE;

        final boolean bothWithinTolerance =
                rightWithinTolerance
                        && leftWithinTolerance;

        /*
         * Both flywheels must remain continuously within tolerance.
         * Leaving tolerance resets the entire settling window.
         */
        if (!bothWithinTolerance) {
            resetReadyState();
            return;
        }

        /*
         * Start the readiness settling window.
         */
        if (readyWindowStartNs < 0L) {
            readyWindowStartNs = nowNs;
            readyToShoot = false;
            return;
        }

        final double elapsedReadyMs =
                (nowNs - readyWindowStartNs)
                        * NANO_TO_MS;

        readyToShoot =
                elapsedReadyMs
                        >= SHOOTER_READY_SETTLE_MS;
    }

    private void resetReadyState() {
        readyToShoot = false;
        readyWindowStartNs = -1L;
    }

    /**
     * Stops both shooter motors and resets all controller state.
     */
    public void stop() {
        targetVelocity = STOP_VELOCITY;
        shooterActive = false;

        requestedRightPower = 0.0;
        requestedLeftPower = 0.0;

        appliedRightPower = 0.0;
        appliedLeftPower = 0.0;

        controllerSaturated = false;

        resetReadyState();

        rightShooter.setPower(0.0);
        leftShooter.setPower(0.0);
    }

    private double sanitizeVelocity(double velocity) {
        if (!Double.isFinite(velocity)) {
            return STOP_VELOCITY;
        }

        return Math.abs(velocity);
    }

    private double sanitizeVoltageCompMultiplier(
            double voltageCompMultiplier
    ) {
        if (!Double.isFinite(voltageCompMultiplier)) {
            return 1.0;
        }

        return Math.max(
                MIN_VOLTAGE_COMP,
                Math.min(
                        MAX_VOLTAGE_COMP,
                        voltageCompMultiplier
                )
        );
    }

    private double clipPower(double power) {
        if (!Double.isFinite(power)) {
            return 0.0;
        }

        return Math.max(
                0.0,
                Math.min(1.0, power)
        );
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public boolean isShooterActive() {
        return shooterActive;
    }

    public boolean isControllerSaturated() {
        return controllerSaturated;
    }

    public boolean isFarZone() {
        return isFarZone;
    }

    public double getAverageVelocity() {
        return 0.5
                * (currentRVel + currentLVel);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRightVelocity() {
        return currentRVel;
    }

    public double getLeftVelocity() {
        return currentLVel;
    }

    public double getRequestedRightPower() {
        return requestedRightPower;
    }

    public double getRequestedLeftPower() {
        return requestedLeftPower;
    }

    public double getAppliedRightPower() {
        return appliedRightPower;
    }

    public double getAppliedLeftPower() {
        return appliedLeftPower;
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        final double rightError =
                targetVelocity - currentRVel;

        final double leftError =
                targetVelocity - currentLVel;

        telemetry.addData(
                "Shooter Controller",
                "Custom FF+P w/ VoltComp Multiplier"
        );

        telemetry.addData(
                "Shooter Active",
                shooterActive
        );

        telemetry.addData(
                "Shooter Ready",
                readyToShoot
        );

        telemetry.addData(
                "Shooter Far Zone",
                isFarZone
        );

        telemetry.addData(
                "Shooter Target",
                targetVelocity
        );

        telemetry.addData(
                "Shooter R Vel",
                currentRVel
        );

        telemetry.addData(
                "Shooter L Vel",
                currentLVel
        );

        telemetry.addData(
                "Shooter Avg Vel",
                getAverageVelocity()
        );

        telemetry.addData(
                "Shooter R Error",
                rightError
        );

        telemetry.addData(
                "Shooter L Error",
                leftError
        );

        telemetry.addData(
                "Shooter Volt Comp",
                lastVoltageCompMultiplier
        );

        telemetry.addData(
                "Shooter R Requested",
                requestedRightPower
        );

        telemetry.addData(
                "Shooter L Requested",
                requestedLeftPower
        );

        telemetry.addData(
                "Shooter R Applied",
                appliedRightPower
        );

        telemetry.addData(
                "Shooter L Applied",
                appliedLeftPower
        );

        telemetry.addData(
                "Shooter Saturated",
                controllerSaturated
        );
    }
}