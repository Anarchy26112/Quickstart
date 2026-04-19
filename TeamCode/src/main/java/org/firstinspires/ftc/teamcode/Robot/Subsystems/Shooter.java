package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    private static final long VOLTAGE_POLL_INTERVAL_NS = 50_000_000L; // 50 ms

    private static final double MIN_CONTROL_DT = 0.001;
    private static final double MAX_D_TERM = 0.20;
    private static final double WRITE_TOLERANCE = 0.001;

    private static final double VOLTAGE_COMP_POWER = 1.1;
    private static final double MIN_VOLTAGE_COMP = 0.85;
    private static final double MAX_VOLTAGE_COMP = 1.45;

    private static final double TARGET_CHANGE_EPSILON = 30.0;
    private static final double LARGE_TARGET_JUMP = 120.0;
    private static final double STOPPED_VELOCITY_EPSILON = 5.0;
    private static final long D_SUPPRESS_AFTER_JUMP_NS = 120_000_000L; // 120 ms

    // ── Voltage cache ─────────────────────────────────────────────────────────
    private double cachedVoltage = NOMINAL_VOLTAGE;
    private long lastVoltagePollNs = 0L;

    // ── Control loop timing ───────────────────────────────────────────────────
    private long lastControlLoopNs = 0L;

    // ── Tuning ────────────────────────────────────────────────────────────────
    private double kV = 0.00037;
    private double kS = 0.02;
    private double kP_FAR = 0.0014;
    private double kP_NEAR = 0.0014;
    private double kD = 0.0;

    // ── Derivative state ──────────────────────────────────────────────────────
    private double lastRVelForD = 0.0;
    private double lastLVelForD = 0.0;
    private boolean derivativeReady = false;
    private long suppressDUntilNs = 0L;

    // ── Write caching ─────────────────────────────────────────────────────────
    private double lastWrittenRPower = Double.NaN;
    private double lastWrittenLPower = Double.NaN;
    private double robotY = 0.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        configureMotor(rightShooter, DcMotor.Direction.FORWARD);
        configureMotor(leftShooter, DcMotor.Direction.REVERSE);

        refreshVoltage();

        long now = System.nanoTime();
        lastVoltagePollNs = now;
        lastControlLoopNs = now;
    }

    public void setRobotY(double y) {
        this.robotY = y;
    }

    private void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    // ── Main update loop ──────────────────────────────────────────────────────
    public void update(long nowNs) {
        currentRVel = Math.abs(rightShooter.getVelocity());
        currentLVel = Math.abs(leftShooter.getVelocity());

        if ((nowNs - lastVoltagePollNs) >= VOLTAGE_POLL_INTERVAL_NS) {
            refreshVoltage();
            lastVoltagePollNs = nowNs;
        }

        calculateAndSetPower(nowNs);
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void setVelocity(double velocity) {
        double v = Math.abs(velocity);

        double deltaR = Math.abs(targetRVelocity - v);
        double deltaL = Math.abs(targetLVelocity - v);

        targetVelocity = v;
        targetRVelocity = v;
        targetLVelocity = v;

        if (deltaR > LARGE_TARGET_JUMP || deltaL > LARGE_TARGET_JUMP) {
            resetDerivativeState();
            suppressDUntilNs = System.nanoTime() + D_SUPPRESS_AFTER_JUMP_NS;
        }
    }

    public void setVelocity(double rightVelocity, double leftVelocity) {
        double r = Math.abs(rightVelocity);
        double l = Math.abs(leftVelocity);

        double deltaR = Math.abs(targetRVelocity - r);
        double deltaL = Math.abs(targetLVelocity - l);

        targetRVelocity = r;
        targetLVelocity = l;
        targetVelocity = 0.5 * (r + l);

        if (deltaR > LARGE_TARGET_JUMP || deltaL > LARGE_TARGET_JUMP) {
            resetDerivativeState();
            suppressDUntilNs = System.nanoTime() + D_SUPPRESS_AFTER_JUMP_NS;
        }
    }

    public void stop() {
        targetVelocity = STOP_VELOCITY;
        targetRVelocity = STOP_VELOCITY;
        targetLVelocity = STOP_VELOCITY;

        resetDerivativeState();
        suppressDUntilNs = System.nanoTime() + D_SUPPRESS_AFTER_JUMP_NS;
    }

    private void resetDerivativeState() {
        lastRVelForD = currentRVel;
        lastLVelForD = currentLVel;
        derivativeReady = false;
        lastControlLoopNs = System.nanoTime();
    }

    // ── Control math ──────────────────────────────────────────────────────────
    private void calculateAndSetPower(long nowNs) {
        double dt = (nowNs - lastControlLoopNs) * 1e-9;
        lastControlLoopNs = nowNs;
        dt = Math.max(dt, MIN_CONTROL_DT);

        if (Math.abs(targetRVelocity) < TARGET_CHANGE_EPSILON &&
                Math.abs(targetLVelocity) < TARGET_CHANGE_EPSILON) {

            currentVoltageComp = 1.0;

            if (currentRVel < STOPPED_VELOCITY_EPSILON && currentLVel < STOPPED_VELOCITY_EPSILON) {
                currentRPower = 0.0;
                currentLPower = 0.0;
                writeMotorPowers(0.0, 0.0);
                return;
            }

            currentRPower = 0.0;
            currentLPower = 0.0;
            writeMotorPowers(0.0, 0.0);
            return;
        }

        // Feedforward
        double ffR = (kV * targetRVelocity) + (kS * Math.signum(targetRVelocity));
        double ffL = (kV * targetLVelocity) + (kS * Math.signum(targetLVelocity));

        // Error
        double errorR = targetRVelocity - currentRVel;
        double errorL = targetLVelocity - currentLVel;

        // Gain scheduling based on Y position
        double activeKP = (robotY < -36.0) ? kP_NEAR : kP_FAR;

        // Proportional
        double pR = activeKP * errorR;
        double pL = activeKP * errorL;

        // Derivative on measurement to reduce derivative kick
        double dR = 0.0;
        double dL = 0.0;

        boolean suppressD = nowNs < suppressDUntilNs;

        if (derivativeReady && !suppressD) {
            double measuredDerivR = (currentRVel - lastRVelForD) / dt;
            double measuredDerivL = (currentLVel - lastLVelForD) / dt;

            dR = -kD * measuredDerivR;
            dL = -kD * measuredDerivL;

            dR = clamp(dR, -MAX_D_TERM, MAX_D_TERM);
            dL = clamp(dL, -MAX_D_TERM, MAX_D_TERM);
        } else {
            derivativeReady = true;
        }

        lastRVelForD = currentRVel;
        lastLVelForD = currentLVel;

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