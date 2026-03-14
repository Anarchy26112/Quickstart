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

    // ── Targets ───────────────────────────────────────────────────────────────
    private double targetVelocity  = 0.0;
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

    private static final double MIN_CONTROL_DT = 0.001;   // 1 ms floor
    private static final double MAX_D_TERM = 0.20;        // optional D clamp
    private static final double WRITE_TOLERANCE = 0.001;

    // Exponential voltage compensation
    private static final double VOLTAGE_COMP_POWER = 1.1;
    private static final double MIN_VOLTAGE_COMP = 0.85;
    private static final double MAX_VOLTAGE_COMP = 1.45;

    // ── Voltage cache ─────────────────────────────────────────────────────────
    private double cachedVoltage = NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    // ── Control loop timing ───────────────────────────────────────────────────
    private final ElapsedTime controlLoopTimer = new ElapsedTime();

    // ── Tuning ────────────────────────────────────────────────────────────────
    private double kV = 0.00037;
    private double kS = 0.02;
    private double kP = 0.0014;
    private double kD = 0.00;

    // ── Derivative state ──────────────────────────────────────────────────────
    private double lastErrorR = 0.0;
    private double lastErrorL = 0.0;
    private boolean derivativeReady = false;

    // ── Write caching ─────────────────────────────────────────────────────────
    private double lastWrittenRPower = -2.0;
    private double lastWrittenLPower = -2.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter  = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

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
        controlLoopTimer.reset();
    }

    // ── Main update loop ──────────────────────────────────────────────────────
    public void update() {
        // Read hardware once
        currentRVel = Math.abs(rightShooter.getVelocity());
        currentLVel = Math.abs(leftShooter.getVelocity());

        // Refresh voltage periodically
        if (voltageTimer.seconds() >= VOLTAGE_POLL_INTERVAL_SEC) {
            refreshVoltage();
            voltageTimer.reset();
        }

        calculateAndSetPower();
    }

    // ── Commands ──────────────────────────────────────────────────────────────
    public void setVelocity(double velocity) {
        double v = Math.abs(velocity);

        targetVelocity  = v;
        targetRVelocity = v;
        targetLVelocity = v;

        resetDerivativeState();
    }

    public void setVelocity(double rightVelocity, double leftVelocity) {
        targetRVelocity = Math.abs(rightVelocity);
        targetLVelocity = Math.abs(leftVelocity);
        targetVelocity = 0.5 * (targetRVelocity + targetLVelocity);

        resetDerivativeState();
    }

    public void stop() {
        targetVelocity  = STOP_VELOCITY;
        targetRVelocity = STOP_VELOCITY;
        targetLVelocity = STOP_VELOCITY;

        resetDerivativeState();
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

        if (targetRVelocity == 0.0 && targetLVelocity == 0.0) {
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

        // Derivative with anti-kick initialization
        double dR = 0.0;
        double dL = 0.0;

        if (derivativeReady) {
            dR = kD * ((errorR - lastErrorR) / dt);
            dL = kD * ((errorL - lastErrorL) / dt);

            // Optional safety clamp for noisy velocity readings
            dR = clamp(dR, -MAX_D_TERM, MAX_D_TERM);
            dL = clamp(dL, -MAX_D_TERM, MAX_D_TERM);
        } else {
            derivativeReady = true;
        }

        lastErrorR = errorR;
        lastErrorL = errorL;

// Power-law voltage compensation
        double voltageRatio = NOMINAL_VOLTAGE / cachedVoltage;

        currentVoltageComp = Math.pow(voltageRatio, VOLTAGE_COMP_POWER);

// Safety clamp
        currentVoltageComp = clamp(currentVoltageComp, MIN_VOLTAGE_COMP, MAX_VOLTAGE_COMP);

        double newRPower = clamp((ffR + pR + dR) * currentVoltageComp, -1.0, 1.0);
        double newLPower = clamp((ffL + pL + dL) * currentVoltageComp, -1.0, 1.0);

        currentRPower = newRPower;
        currentLPower = newLPower;

        writeMotorPowers(newRPower, newLPower);
    }

    private void writeMotorPowers(double rightPower, double leftPower) {
        if (Math.abs(lastWrittenRPower - rightPower) > WRITE_TOLERANCE) {
            rightShooter.setPower(rightPower);
            lastWrittenRPower = rightPower;
        }

        if (Math.abs(lastWrittenLPower - leftPower) > WRITE_TOLERANCE) {
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

    // ── Getters / tuning ──────────────────────────────────────────────────────
    public double getRightVelocity() {
        return currentRVel;
    }

    public double getLeftVelocity() {
        return currentLVel;
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

    public void setTunings(double kV, double kS, double kP, double kD) {
        this.kV = kV;
        this.kS = kS;
        this.kP = kP;
        this.kD = kD;
    }

    public void sendTelemetry() {
        telemetry.addData("Shooter | Battery (V)", "%.2f", cachedVoltage);
        telemetry.addData("Shooter | Volt Comp", "%.3f", currentVoltageComp);
        telemetry.addData("Shooter | Target", "%.1f", targetVelocity);
        telemetry.addData("Shooter | Target R", "%.1f", targetRVelocity);
        telemetry.addData("Shooter | Target L", "%.1f", targetLVelocity);
        telemetry.addData("Shooter | Actual R", "%.1f", currentRVel);
        telemetry.addData("Shooter | Actual L", "%.1f", currentLVel);
        telemetry.addData("Shooter | Power R", "%.3f", currentRPower);
        telemetry.addData("Shooter | Power L", "%.3f", currentLPower);
        telemetry.addData("Shooter | kV", "%.6f", kV);
        telemetry.addData("Shooter | kS", "%.6f", kS);
        telemetry.addData("Shooter | kP", "%.6f", kP);
        telemetry.addData("Shooter | kD", "%.6f", kD);
    }
}
