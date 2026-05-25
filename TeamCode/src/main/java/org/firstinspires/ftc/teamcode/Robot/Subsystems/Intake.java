package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Intake {

    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    private final Telemetry telemetry;

    private double intakePower = 0.0;
    private double transferPower = 0.0;
    private double lastWrittenIntakePower = -2.0;
    private double lastWrittenTransferPower = -2.0;

    private static final double WRITE_TOLERANCE = 0.004;
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotorEx.class, HW_INTAKE);
        transfer = hardwareMap.get(DcMotorEx.class, HW_TRANSFER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(double power) {
        setIntakePower(power < 0.0 ? -power : power);
    }

    public void spit(double power) {
        setIntakePower(power < 0.0 ? power : -power);
    }

    public void stopIntake() {
        setIntakePower(STOP_POWER);
    }

    private void setIntakePower(double power) {
        if (power == -0.0) power = 0.0;
        intakePower = power;

        // Inline comparison: avoid function call overhead
        if (shouldWriteIntakePower(power, lastWrittenIntakePower)) {
            intake.setPower(power);
            lastWrittenIntakePower = power;
        }
    }

    private boolean shouldWriteIntakePower(double newPower, double lastPower) {
        if (newPower == 0.0 && lastPower != 0.0) return true;
        if (newPower != 0.0) {
            double diff = newPower - lastPower;
            return (diff > WRITE_TOLERANCE || diff < -WRITE_TOLERANCE);
        }
        return false;
    }

    public boolean isIntakeRunning() {
        return intakePower > POWER_THRESHOLD || intakePower < -POWER_THRESHOLD;
    }

    public String getIntakeState() {
        if (intakePower > POWER_THRESHOLD) return "Intaking";
        if (intakePower < -POWER_THRESHOLD) return "Spitting";
        return "Stopped";
    }

    public double getIntakePower() {
        return intakePower;
    }

    public void transferIn(double power) {
        setTransferPower(power < 0.0 ? -power : power);
    }

    public void transferOut(double power) {
        setTransferPower(power < 0.0 ? power : -power);
    }

    public void stopTransfer() {
        setTransferPower(STOP_POWER);
    }

    private void setTransferPower(double power) {
        if (power == -0.0) power = 0.0;
        transferPower = power;

        if (shouldWriteTransferPower(power, lastWrittenTransferPower)) {
            transfer.setPower(power);
            lastWrittenTransferPower = power;
        }
    }

    private boolean shouldWriteTransferPower(double newPower, double lastPower) {
        if (newPower == 0.0 && lastPower != 0.0) return true;
        if (newPower != 0.0) {
            double diff = newPower - lastPower;
            return (diff > WRITE_TOLERANCE || diff < -WRITE_TOLERANCE);
        }
        return false;
    }

    public boolean isTransferRunning() {
        return transferPower > POWER_THRESHOLD || transferPower < -POWER_THRESHOLD;
    }

    public String getTransferState() {
        if (transferPower > POWER_THRESHOLD) return "Forward";
        if (transferPower < -POWER_THRESHOLD) return "Reverse";
        return "Stopped";
    }

    public double getTransferPower() {
        return transferPower;
    }

    public void intakeBoth(double power) {
        final double p = power < 0.0 ? -power : power;
        setIntakePower(p);
        setTransferPower(p);
    }

    public void spitBoth(double power) {
        final double p = power < 0.0 ? power : -power;
        setIntakePower(p);
        setTransferPower(p);
    }

    public void stopAll() {
        setIntakePower(STOP_POWER);
        setTransferPower(STOP_POWER);
    }

    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Intake", getIntakeState());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Transfer", getTransferState());
        telemetry.addData("Transfer Power", transferPower);
    }
}