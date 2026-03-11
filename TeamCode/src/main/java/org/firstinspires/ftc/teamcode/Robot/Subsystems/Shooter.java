/*
package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final Telemetry telemetry;

    // Target velocities in ticks per second
    private double targetVelocity = 0.0;
    private double targetRVelocity = 0.0;
    private double targetLVelocity = 0.0;

    private static final double STOP_VELOCITY = 0.0;
    private static final double VELOCITY_THRESHOLD = 50.0;

    // ---- YOUR TUNED PIDF VALUES ----
    // Put these in HamiltonParams if you want.
    private static final double SHOOTER_kP = 60.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 12;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        // Directions (mirrored setup)
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        // Flywheels: FLOAT is typical
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset + velocity control mode
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- APPLY CUSTOM PIDF TO BUILT-IN VELOCITY LOOP ----
        applyVelocityPIDF(new PIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        ));
    }

    private void applyVelocityPIDF(PIDFCoefficients pidf) {
        // This sets the internal motor controller coefficients for velocity control
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // Set shooter velocity (ticks/sec) for both motors
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        targetRVelocity = velocity;
        targetLVelocity = velocity;

        rightShooter.setVelocity(velocity);
        leftShooter.setVelocity(velocity);
    }

    // Independent set (if you still want it)
    public void setRVelocity(double velocity) {
        targetRVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        rightShooter.setVelocity(velocity);
    }

    public void setLVelocity(double velocity) {
        targetLVelocity = velocity;
        targetVelocity = (targetRVelocity + targetLVelocity) / 2.0;
        leftShooter.setVelocity(velocity);
    }

    // Legacy power -> velocity
    public void setPower(double power) {
        setVelocity(power * SHOOTER_MAX_VELOCITY);
    }

    public void spin(double velocity) {
        setVelocity(Math.abs(velocity));
    }

    public void stop() {
        setVelocity(STOP_VELOCITY);
    }

    public boolean isRunning() {
        return getRightVelocity() > VELOCITY_THRESHOLD ||
                getLeftVelocity() > VELOCITY_THRESHOLD;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetRVelocity() {
        return targetRVelocity;
    }

    public double getTargetLVelocity() {
        return targetLVelocity;
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getAverageVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2.0;
    }

    public double getVelocityError() {
        return targetVelocity - getAverageVelocity();
    }
}
 */
package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final Iterable<VoltageSensor> voltageSensors;
    private final Telemetry telemetry;

    // ── Velocity targets ──────────────────────────────────────────────────────
    private double targetVelocity  = 0.0;
    private double targetRVelocity = 0.0;
    private double targetLVelocity = 0.0;

    private static final double STOP_VELOCITY      = 0.0;
    private static final double VELOCITY_THRESHOLD = 50.0;

    // ── Voltage compensation ──────────────────────────────────────────────────
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE     = 8.0;
    private static final double MAX_VOLTAGE     = 14.0;

    // More aggressive than linear compensation at lower voltage
    private static final double VOLTAGE_COMP_EXPONENT = 1.15;

    private double cachedVoltage = NOMINAL_VOLTAGE;

    // ── PIDF ──────────────────────────────────────────────────────────────────
    // Tune these on the robot
    private static final double SHOOTER_kP = 75.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 12.4;

    // ── I2C / coefficient write caching ──────────────────────────────────────
    private double lastAppliedF = SHOOTER_kF;
    private static final double KF_WRITE_THRESHOLD = 0.03;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_POLL_INTERVAL_SEC = 0.10;

    // ─────────────────────────────────────────────────────────────────────────

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter  = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        voltageSensors = hardwareMap.voltageSensor;

        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        refreshVoltage();
        voltageTimer.reset();

        applyCompensatedPIDF(true);
    }

    // ── Update Loop ───────────────────────────────────────────────────────────

    public void update() {
        if (voltageTimer.seconds() >= VOLTAGE_POLL_INTERVAL_SEC) {
            refreshVoltage();
            voltageTimer.reset();
        }

        applyCompensatedPIDF(false);
        sendTelemetry();
    }

    // ── Control Methods ───────────────────────────────────────────────────────

    public void setVelocity(double velocity) {
        targetVelocity  = Math.abs(velocity);
        targetRVelocity = Math.abs(velocity);
        targetLVelocity = Math.abs(velocity);

        rightShooter.setVelocity(targetRVelocity);
        leftShooter.setVelocity(targetLVelocity);
    }

    public void setRVelocity(double velocity) {
        targetRVelocity = Math.abs(velocity);
        targetVelocity  = (targetRVelocity + targetLVelocity) / 2.0;

        rightShooter.setVelocity(targetRVelocity);
    }

    public void setLVelocity(double velocity) {
        targetLVelocity = Math.abs(velocity);
        targetVelocity  = (targetRVelocity + targetLVelocity) / 2.0;

        leftShooter.setVelocity(targetLVelocity);
    }

    public void setPower(double power) {
        setVelocity(power * SHOOTER_MAX_VELOCITY);
    }

    public void spin(double velocity) {
        setVelocity(velocity);
    }

    public void stop() {
        targetVelocity  = STOP_VELOCITY;
        targetRVelocity = STOP_VELOCITY;
        targetLVelocity = STOP_VELOCITY;

        rightShooter.setVelocity(STOP_VELOCITY);
        leftShooter.setVelocity(STOP_VELOCITY);
    }

    // ── State Queries ─────────────────────────────────────────────────────────

    public boolean isRunning() {
        return Math.abs(getRightVelocity()) > VELOCITY_THRESHOLD ||
                Math.abs(getLeftVelocity())  > VELOCITY_THRESHOLD;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(targetRVelocity - getRightVelocity()) < VELOCITY_THRESHOLD &&
                Math.abs(targetLVelocity - getLeftVelocity())  < VELOCITY_THRESHOLD;
    }

    public double getTargetVelocity()  { return targetVelocity; }
    public double getTargetRVelocity() { return targetRVelocity; }
    public double getTargetLVelocity() { return targetLVelocity; }

    public double getRightVelocity() { return rightShooter.getVelocity(); }
    public double getLeftVelocity()  { return leftShooter.getVelocity(); }

    public double getAverageVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2.0;
    }

    public double getVelocityError() {
        return targetVelocity - getAverageVelocity();
    }

    public double getRightVelocityError() {
        return targetRVelocity - getRightVelocity();
    }

    public double getLeftVelocityError() {
        return targetLVelocity - getLeftVelocity();
    }

    public double getVoltage() {
        return cachedVoltage;
    }

    public double getCompensatedKF() {
        return lastAppliedF;
    }

    // ── Internal Helpers ──────────────────────────────────────────────────────

    private void reapplyTargetVelocity() {
        // Helps ensure the controller keeps the current targets after PIDF updates
        rightShooter.setVelocity(targetRVelocity);
        leftShooter.setVelocity(targetLVelocity);
    }

    private void refreshVoltage() {
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();

            if (!Double.isNaN(voltage) && voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        if (result == Double.POSITIVE_INFINITY) {
            cachedVoltage = NOMINAL_VOLTAGE;
            return;
        }

        cachedVoltage = clamp(result, MIN_VOLTAGE, MAX_VOLTAGE);
    }

    private double computeCompensatedKF(double voltage) {
        double ratio = NOMINAL_VOLTAGE / voltage;

        // Very mild exponential compensation
        double exponent = 1.4;

        return SHOOTER_kF * Math.pow(ratio, exponent);
    }


    private void applyCompensatedPIDF(boolean force) {
        double compensatedF = computeCompensatedKF(cachedVoltage);

        if (!force && Math.abs(compensatedF - lastAppliedF) < KF_WRITE_THRESHOLD) {
            return;
        }

        PIDFCoefficients pidf = new PIDFCoefficients(
                SHOOTER_kP,
                SHOOTER_kI,
                SHOOTER_kD,
                compensatedF
        );

        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        lastAppliedF = compensatedF;
        reapplyTargetVelocity();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void sendTelemetry() {
        telemetry.addData("Shooter | Battery (V)", "%.2f", cachedVoltage);
        telemetry.addData("Shooter | kP", "%.2f", SHOOTER_kP);
        telemetry.addData("Shooter | kF (base)", "%.3f", SHOOTER_kF);
        telemetry.addData("Shooter | kF (comp)", "%.3f", lastAppliedF);
        telemetry.addData("Shooter | Target R", "%.1f", targetRVelocity);
        telemetry.addData("Shooter | Target L", "%.1f", targetLVelocity);
        telemetry.addData("Shooter | Actual R", "%.1f", getRightVelocity());
        telemetry.addData("Shooter | Actual L", "%.1f", getLeftVelocity());
        telemetry.addData("Shooter | Err R", "%.1f", getRightVelocityError());
        telemetry.addData("Shooter | Err L", "%.1f", getLeftVelocityError());
    }
}
