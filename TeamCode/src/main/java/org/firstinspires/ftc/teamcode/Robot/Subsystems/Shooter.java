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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Shooter {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    // ── Velocity Targets ──────────────────────────────────────────────────────
    private double targetVelocity  = 0.0;
    private double targetRVelocity = 0.0;
    private double targetLVelocity = 0.0;
    // Cached hardware reads to save loop time
    private double currentRVel = 0.0;
    private double currentLVel = 0.0;
    private double currentRPower = 0.0;
    private double currentLPower = 0.0;

    private static final double STOP_VELOCITY      = 0.0;
    private static final double VELOCITY_THRESHOLD = 50.0;

    // ── Voltage Compensation ──────────────────────────────────────────────────
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE     = 8.0;
    private static final double MAX_VOLTAGE     = 14.0;
    private double cachedVoltage = NOMINAL_VOLTAGE;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_POLL_INTERVAL_SEC = 0.10;

    // ── Custom Feedforward & PID Tuning ───────────────────────────────────────
    // Feedforward (Predictive)
    private double kV = 0.00037;
    private double kS = 0.02;
    private double kP = 0.0018;

    // ─────────────────────────────────────────────────────────────────────────

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter  = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        // Cache a single voltage sensor to avoid garbage collection overhead
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        refreshVoltage();
        voltageTimer.reset();
    }

    // ── Update Loop ───────────────────────────────────────────────────────────

    public void update() {
        // 1. Read hardware exactly ONCE per loop
        currentRVel = Math.abs(rightShooter.getVelocity());
        currentLVel = Math.abs(leftShooter.getVelocity());

        // 2. Poll voltage on a timer (Your existing, excellent logic)
        if (voltageTimer.seconds() >= VOLTAGE_POLL_INTERVAL_SEC) {
            refreshVoltage();
            voltageTimer.reset();
        }

        // 3. Do the math
        calculateAndSetPower();
    }

    // ── Control Methods ───────────────────────────────────────────────────────

    public void setVelocity(double velocity) {
        targetVelocity  = Math.abs(velocity);
        targetRVelocity = Math.abs(velocity);
        targetLVelocity = Math.abs(velocity);
    }

    public void stop() {
        targetVelocity  = STOP_VELOCITY;
        targetRVelocity = STOP_VELOCITY;
        targetLVelocity = STOP_VELOCITY;

        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    // ── Core Math ─────────────────────────────────────────────────────────────

    private void calculateAndSetPower() {
        if (targetRVelocity == 0 && targetLVelocity == 0) {
            currentRPower = 0;
            currentLPower = 0;
            rightShooter.setPower(0);
            leftShooter.setPower(0);
            return;
        }

        // Feedforward
        double ffR = (kV * targetRVelocity) + (kS * Math.signum(targetRVelocity));
        double ffL = (kV * targetLVelocity) + (kS * Math.signum(targetLVelocity));

        // Proportional (Using the CACHED velocities)
        double errorR = targetRVelocity - currentRVel;
        double errorL = targetLVelocity - currentLVel;

        double pR = kP * errorR;
        double pL = kP * errorL;

        double voltageCompensationRatio = NOMINAL_VOLTAGE / cachedVoltage;

        // Cache the calculated power so telemetry doesn't have to ask the motor
        currentRPower = clamp((ffR + pR) * voltageCompensationRatio, -1.0, 1.0);
        currentLPower = clamp((ffL + pL) * voltageCompensationRatio, -1.0, 1.0);

        rightShooter.setPower(currentRPower);
        leftShooter.setPower(currentLPower);
    }

    private void refreshVoltage() {
        double voltage = batteryVoltageSensor.getVoltage();
        if (!Double.isNaN(voltage) && voltage > 0) {
            cachedVoltage = clamp(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ... (State Queries and Telemetry methods remain exactly the same) ...
    public double getRightVelocity() { return rightShooter.getVelocity(); }
    public double getLeftVelocity()  { return leftShooter.getVelocity(); }

    private void sendTelemetry() {
        telemetry.addData("Shooter | Battery (V)", "%.2f", cachedVoltage);
        telemetry.addData("Shooter | Target", "%.1f", targetVelocity);
        telemetry.addData("Shooter | Target R", "%.1f", targetRVelocity);
        telemetry.addData("Shooter | Actual R", "%.1f", currentRVel);
        telemetry.addData("Shooter | Actual L", "%.1f", currentLVel);
        telemetry.addData("Shooter | Power R", "%.2f", currentRPower);
        telemetry.addData("Shooter | Power L", "%.2f", currentLPower);
        telemetry.addData("Shooter | kV", "%.6f", kV);
        telemetry.addData("Shooter | kS", "%.6f", kS);
        telemetry.addData("Shooter | kP", "%.6f", kP);
    }
    public void setTunings(double kV, double kS, double kP) {
        this.kV = kV;
        this.kS = kS;
        this.kP = kP;
    }

    public double getKV() { return kV; }
    public double getKS() { return kS; }
    public double getKP() { return kP; }

    public double getTargetVelocity() {
        return targetVelocity;
    }


}