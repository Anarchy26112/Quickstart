package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_RIGHT_SHOOTER;

public class Shooter {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    // ── Targets ───────────────────────────────────────────────────────────────
    private double targetVelocity = 0.0;
    private double targetRVelocity = 0.0;
    private double targetLVelocity = 0.0;

    // ── Cached sensor / output state ──────────────────────────────────────────
    private double currentRVel = 0.0;
    private double currentLVel = 0.0;
    private double currentRPower = 0.0;
    private double currentLPower = 0.0;
    private double currentVoltageComp = 1.0;

    // ── Constants ─────────────────────────────────────────────────────────────
    private static final double STOP_VELOCITY = 0.0;

    private static final double NOMINAL_VOLTAGE = 12.2;
    private static final double MIN_VOLTAGE = 8.0;
    private static final double MAX_VOLTAGE = 14.0;
    private static final double VOLTAGE_POLL_INTERVAL_SEC = 0.05;

    private static final double MIN_CONTROL_DT = 0.001;
    private static final double MAX_D_TERM = 0.20;
    private static final double WRITE_TOLERANCE = 0.001;

    private static final double VOLTAGE_COMP_POWER = 1.1;
    private static final double MIN_VOLTAGE_COMP = 0.85;
    private static final double MAX_VOLTAGE_COMP = 1.45;

    // Only reset derivative when target meaningfully changes
    private static final double TARGET_CHANGE_EPSILON = 1.0; // ticks/sec

    // ── Voltage cache ─────────────────────────────────────────────────────────
    private double cachedVoltage = NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    // ── Control loop timing ───────────────────────────────────────────────────
    private final ElapsedTime controlLoopTimer = new ElapsedTime();

    // ── Tuning ────────────────────────────────────────────────────────────────
    private double kV = 0.00037;
    private double kS = 0.02;
    private double kP = 0.0033;
    private double kD = 0.0000;

    // ── Derivative state ──────────────────────────────────────────────────────
    private double lastErrorR = 0.0;
    private double lastErrorL = 0.0;
    private boolean derivativeReady = false;

    // ── Write caching ─────────────────────────────────────────────────────────
    private double lastWrittenRPower = Double.NaN;
    private double lastWrittenLPower = Double.NaN;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        configureMotor(rightShooter, DcMotor.Direction.FORWARD);
        configureMotor(leftShooter, DcMotor.Direction.REVERSE);

        refreshVoltage();
        voltageTimer.reset();
        controlLoopTimer.reset();
    }

    private void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    // ── Main update loop ──────────────────────────────────────────────────────
    public void update() {
        currentRVel = Math.abs(rightShooter.getVelocity());
        currentLVel = Math.abs(leftShooter.getVelocity());

        if (voltageTimer.seconds() >= VOLTAGE_POLL_INTERVAL_SEC) {
            refreshVoltage();
            voltageTimer.reset();
        }

        calculateAndSetPower();
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void setVelocity(double velocity) {
        double v = Math.abs(velocity);

        boolean changed =
                Math.abs(targetRVelocity - v) > TARGET_CHANGE_EPSILON ||
                        Math.abs(targetLVelocity - v) > TARGET_CHANGE_EPSILON;

        targetVelocity = v;
        targetRVelocity = v;
        targetLVelocity = v;

        if (changed) {
            resetDerivativeState();
        }
    }

    public void setVelocity(double rightVelocity, double leftVelocity) {
        double r = Math.abs(rightVelocity);
        double l = Math.abs(leftVelocity);

        boolean changed =
                Math.abs(targetRVelocity - r) > TARGET_CHANGE_EPSILON ||
                        Math.abs(targetLVelocity - l) > TARGET_CHANGE_EPSILON;

        targetRVelocity = r;
        targetLVelocity = l;
        targetVelocity = 0.5 * (r + l);

        if (changed) {
            resetDerivativeState();
        }
    }

    public void stop() {
        boolean changed =
                Math.abs(targetRVelocity) > TARGET_CHANGE_EPSILON ||
                        Math.abs(targetLVelocity) > TARGET_CHANGE_EPSILON;

        targetVelocity = STOP_VELOCITY;
        targetRVelocity = STOP_VELOCITY;
        targetLVelocity = STOP_VELOCITY;

        if (changed) {
            resetDerivativeState();
        }
    }

    private void resetDerivativeState() {
        lastErrorR = 0.0;
        lastErrorL = 0.0;
        derivativeReady = false;
        controlLoopTimer.reset();
    }

    // ── Control math ──────────────────────────────────────────────────────────
    private void calculateAndSetPower() {
        double dt = controlLoopTimer.seconds();
        controlLoopTimer.reset();
        dt = Math.max(dt, MIN_CONTROL_DT);

        if (Math.abs(targetRVelocity) < TARGET_CHANGE_EPSILON &&
                Math.abs(targetLVelocity) < TARGET_CHANGE_EPSILON) {
            currentRPower = 0.0;
            currentLPower = 0.0;
            currentVoltageComp = 1.0;

            writeMotorPowers(0.0, 0.0);
            return;
        }

        // Feedforward
        double ffR = (kV * targetRVelocity) + (kS * Math.signum(targetRVelocity));
        double ffL = (kV * targetLVelocity) + (kS * Math.signum(targetLVelocity));

        // Error
        double errorR = targetRVelocity - currentRVel;
        double errorL = targetLVelocity - currentLVel;

        // Proportional
        double pR = kP * errorR;
        double pL = kP * errorL;

        // Derivative
        double dR = 0.0;
        double dL = 0.0;

        if (derivativeReady) {
            dR = kD * ((errorR - lastErrorR) / dt);
            dL = kD * ((errorL - lastErrorL) / dt);

            dR = clamp(dR, -MAX_D_TERM, MAX_D_TERM);
            dL = clamp(dL, -MAX_D_TERM, MAX_D_TERM);
        } else {
            derivativeReady = true;
        }

        lastErrorR = errorR;
        lastErrorL = errorL;

        // Voltage compensation
        double voltageRatio = NOMINAL_VOLTAGE / cachedVoltage;
        currentVoltageComp = Math.pow(voltageRatio, VOLTAGE_COMP_POWER);
        currentVoltageComp = clamp(currentVoltageComp, MIN_VOLTAGE_COMP, MAX_VOLTAGE_COMP);

        double newRPower = clamp((ffR + pR + dR) * currentVoltageComp, -1.0, 1.0);
        double newLPower = clamp((ffL + pL + dL) * currentVoltageComp, -1.0, 1.0);

        currentRPower = newRPower;
        currentLPower = newLPower;

        writeMotorPowers(newRPower, newLPower);
    }

    private void writeMotorPowers(double rightPower, double leftPower) {
        if (Double.isNaN(lastWrittenRPower) ||
                Math.abs(lastWrittenRPower - rightPower) > WRITE_TOLERANCE) {
            rightShooter.setPower(rightPower);
            lastWrittenRPower = rightPower;
        }

        if (Double.isNaN(lastWrittenLPower) ||
                Math.abs(lastWrittenLPower - leftPower) > WRITE_TOLERANCE) {
            leftShooter.setPower(leftPower);
            lastWrittenLPower = leftPower;
        }
    }

    private void refreshVoltage() {
        double voltage = batteryVoltageSensor.getVoltage();
        if (!Double.isNaN(voltage) && voltage > 0.0) {
            cachedVoltage = clamp(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public double getRightVelocity() {
        return currentRVel;
    }

    public double getLeftVelocity() {
        return currentLVel;
    }

    public double getAverageVelocity() {
        return 0.5 * (currentRVel + currentLVel);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetRightVelocity() {
        return targetRVelocity;
    }

    public double getTargetLeftVelocity() {
        return targetLVelocity;
    }

    public double getRightPower() {
        return currentRPower;
    }

    public double getLeftPower() {
        return currentLPower;
    }

    public double getVoltageCompensation() {
        return currentVoltageComp;
    }

    public double getBatteryVoltage() {
        return cachedVoltage;
    }

    public double getKV() {
        return kV;
    }

    public double getKS() {
        return kS;
    }

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    // ── Tuners ────────────────────────────────────────────────────────────────
    public void setKV(double kV) {
        this.kV = kV;
    }

    public void setKS(double kS) {
        this.kS = kS;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────
    public void telemetry() {
        telemetry.addData("Shooter Target", "%.1f", targetVelocity);
        telemetry.addData("Shooter Vel R", "%.1f", currentRVel);
        telemetry.addData("Shooter Vel L", "%.1f", currentLVel);
        telemetry.addData("Shooter Power R", "%.3f", currentRPower);
        telemetry.addData("Shooter Power L", "%.3f", currentLPower);
        telemetry.addData("Battery Voltage", "%.2f", cachedVoltage);
        telemetry.addData("Voltage Comp", "%.3f", currentVoltageComp);
    }
}