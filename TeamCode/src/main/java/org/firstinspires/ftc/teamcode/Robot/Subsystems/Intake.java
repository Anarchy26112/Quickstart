package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class Intake {

    private final DcMotor intake;
    private final DcMotor transfer;
    private final Telemetry telemetry;

    // Logical commanded power
    private double intakePower = 0.0;
    private double transferPower = 0.0;

    // Last hardware-written power (write caching)
    private double lastWrittenIntakePower = -2.0;
    private double lastWrittenTransferPower = -2.0;
    private static final double WRITE_TOLERANCE = 0.001;

    // Constants
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hardwareMap.get(DcMotor.class, HW_INTAKE);
        transfer = hardwareMap.get(DcMotor.class, HW_TRANSFER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ===================== INTAKE (INDIVIDUAL) ===================== */

    public void intake() {
        setIntakePower(INTAKE_POWER);
    }

    public void intake(double power) {
        setIntakePower(Math.abs(power));
    }

    public void spit() {
        setIntakePower(-SPIT_POWER);
    }

    public void spit(double power) {
        setIntakePower(-Math.abs(power));
    }

    public void stopIntake() {
        setIntakePower(STOP_POWER);
    }

    private void setIntakePower(double power) {
        intakePower = power;

        if (Math.abs(lastWrittenIntakePower - power) > WRITE_TOLERANCE) {
            intake.setPower(power);
            lastWrittenIntakePower = power;
        }
    }

    public boolean isIntakeRunning() {
        return Math.abs(intakePower) > POWER_THRESHOLD;
    }

    public String getIntakeState() {
        if (intakePower > POWER_THRESHOLD) return "Intaking";
        if (intakePower < -POWER_THRESHOLD) return "Spitting";
        return "Stopped";
    }

    public double getIntakePower() {
        return intakePower;
    }

    /* ===================== TRANSFER (INDIVIDUAL) ===================== */

    public void transferIn() {
        setTransferPower(TRANSFER_POWER);
    }

    public void transferIn(double power) {
        setTransferPower(Math.abs(power));
    }

    public void transferOut() {
        setTransferPower(-TRANSFER_POWER);
    }

    public void transferOut(double power) {
        setTransferPower(-Math.abs(power));
    }

    public void stopTransfer() {
        setTransferPower(STOP_POWER);
    }

    private void setTransferPower(double power) {
        transferPower = power;

        if (Math.abs(lastWrittenTransferPower - power) > WRITE_TOLERANCE) {
            transfer.setPower(power);
            lastWrittenTransferPower = power;
        }
    }

    public boolean isTransferRunning() {
        return Math.abs(transferPower) > POWER_THRESHOLD;
    }

    public String getTransferState() {
        if (transferPower > POWER_THRESHOLD) return "Forward";
        if (transferPower < -POWER_THRESHOLD) return "Reverse";
        return "Stopped";
    }

    public double getTransferPower() {
        return transferPower;
    }

    /* ===================== COMBINED (BOTH MOTORS) ===================== */

    public void intakeBoth() {
        setIntakePower(INTAKE_POWER);
        setTransferPower(TRANSFER_POWER);
    }

    public void intakeBoth(double power) {
        double p = Math.abs(power);
        setIntakePower(p);
        setTransferPower(p);
    }

    public void spitBoth() {
        setIntakePower(-SPIT_POWER);
        setTransferPower(-SPIT_POWER);
    }

    public void spitBoth(double power) {
        double p = Math.abs(power);
        setIntakePower(-p);
        setTransferPower(-p);
    }

    public void stopAll() {
        stopIntake();
        stopTransfer();
    }

    /* ===================== TELEMETRY ===================== */

    public void telemetry() {
        telemetry.addData("Intake", getIntakeState());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Transfer", getTransferState());
        telemetry.addData("Transfer Power", transferPower);
    }
}