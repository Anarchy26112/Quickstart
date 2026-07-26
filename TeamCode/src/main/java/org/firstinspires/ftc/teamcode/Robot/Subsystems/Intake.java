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

    /*
     * Requested powers before voltage compensation.
     *
     * These values are used for state reporting and are
     * reapplied whenever the voltage multiplier changes.
     */
    private double intakePower = 0.0;
    private double transferPower = 0.0;

    /*
     * Actual compensated powers most recently written
     * to the motor controllers.
     */
    private double appliedIntakePower = 0.0;
    private double appliedTransferPower = 0.0;

    private double lastWrittenIntakePower = -2.0;
    private double lastWrittenTransferPower = -2.0;

    /*
     * Supplied by TeleopBlue using its cached battery
     * voltage calculation.
     */
    private double voltageCompensation = 1.0;

    private static final double WRITE_TOLERANCE = 0.004;
    private static final double STOP_POWER = 0.0;
    private static final double POWER_THRESHOLD = 0.01;

    /*
     * Defensive limits matching the voltage compensation
     * limits used in TeleopBlue.
     */
    private static final double MIN_VOLTAGE_COMPENSATION = 0.85;
    private static final double MAX_VOLTAGE_COMPENSATION = 1.45;

    public Intake(
            HardwareMap hardwareMap,
            Telemetry telemetry
    ) {
        this.telemetry = telemetry;

        intake = hardwareMap.get(
                DcMotorEx.class,
                HW_INTAKE
        );

        transfer = hardwareMap.get(
                DcMotorEx.class,
                HW_TRANSFER
        );

        intake.setDirection(
                DcMotor.Direction.REVERSE
        );

        transfer.setDirection(
                DcMotor.Direction.REVERSE
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        transfer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        transfer.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    /**
     * Sets the current battery-voltage compensation
     * multiplier.
     *
     * The current intake and transfer requests are
     * immediately recalculated so compensation can change
     * while either motor is already running.
     */
    public void setVoltageCompensation(
            double compensation
    ) {
        if (!Double.isFinite(compensation)
                || compensation <= 0.0) {

            compensation = 1.0;
        }

        compensation = clamp(
                compensation,
                MIN_VOLTAGE_COMPENSATION,
                MAX_VOLTAGE_COMPENSATION
        );

        if (Math.abs(
                compensation - voltageCompensation
        ) < 1.0e-6) {

            return;
        }

        voltageCompensation = compensation;

        writeIntakePower();
        writeTransferPower();
    }

    public double getVoltageCompensation() {
        return voltageCompensation;
    }

    public void intake(double power) {
        setIntakePower(
                power < 0.0
                        ? -power
                        : power
        );
    }

    public void spit(double power) {
        setIntakePower(
                power < 0.0
                        ? power
                        : -power
        );
    }

    public void stopIntake() {
        setIntakePower(
                STOP_POWER
        );
    }

    private void setIntakePower(double power) {
        if (!Double.isFinite(power)) {
            power = STOP_POWER;
        }

        if (power == -0.0) {
            power = 0.0;
        }

        intakePower = power;

        writeIntakePower();
    }

    private void writeIntakePower() {
        final double compensatedPower =
                calculateCompensatedPower(
                        intakePower
                );

        if (shouldWritePower(
                compensatedPower,
                lastWrittenIntakePower
        )) {
            intake.setPower(
                    compensatedPower
            );

            lastWrittenIntakePower =
                    compensatedPower;

            appliedIntakePower =
                    compensatedPower;
        }
    }

    public boolean isIntakeRunning() {
        return intakePower > POWER_THRESHOLD
                || intakePower < -POWER_THRESHOLD;
    }

    public String getIntakeState() {
        if (intakePower > POWER_THRESHOLD) {
            return "Intaking";
        }

        if (intakePower < -POWER_THRESHOLD) {
            return "Spitting";
        }

        return "Stopped";
    }

    /**
     * Returns the requested intake power before voltage
     * compensation.
     */
    public double getIntakePower() {
        return intakePower;
    }

    /**
     * Returns the compensated intake power most recently
     * written to the motor.
     */
    public double getAppliedIntakePower() {
        return appliedIntakePower;
    }

    public void transferIn(double power) {
        setTransferPower(
                power < 0.0
                        ? -power
                        : power
        );
    }

    public void transferOut(double power) {
        setTransferPower(
                power < 0.0
                        ? power
                        : -power
        );
    }

    public void stopTransfer() {
        setTransferPower(
                STOP_POWER
        );
    }

    private void setTransferPower(double power) {
        if (!Double.isFinite(power)) {
            power = STOP_POWER;
        }

        if (power == -0.0) {
            power = 0.0;
        }

        transferPower = power;

        writeTransferPower();
    }

    private void writeTransferPower() {
        final double compensatedPower =
                calculateCompensatedPower(
                        transferPower
                );

        if (shouldWritePower(
                compensatedPower,
                lastWrittenTransferPower
        )) {
            transfer.setPower(
                    compensatedPower
            );

            lastWrittenTransferPower =
                    compensatedPower;

            appliedTransferPower =
                    compensatedPower;
        }
    }

    public boolean isTransferRunning() {
        return transferPower > POWER_THRESHOLD
                || transferPower < -POWER_THRESHOLD;
    }

    public String getTransferState() {
        if (transferPower > POWER_THRESHOLD) {
            return "Forward";
        }

        if (transferPower < -POWER_THRESHOLD) {
            return "Reverse";
        }

        return "Stopped";
    }

    /**
     * Returns the requested transfer power before voltage
     * compensation.
     */
    public double getTransferPower() {
        return transferPower;
    }

    /**
     * Returns the compensated transfer power most recently
     * written to the motor.
     */
    public double getAppliedTransferPower() {
        return appliedTransferPower;
    }

    public void intakeBoth(double power) {
        final double p =
                power < 0.0
                        ? -power
                        : power;

        setIntakePower(p);
        setTransferPower(p);
    }

    public void spitBoth(double power) {
        final double p =
                power < 0.0
                        ? power
                        : -power;

        setIntakePower(p);
        setTransferPower(p);
    }

    public void stopAll() {
        setIntakePower(
                STOP_POWER
        );

        setTransferPower(
                STOP_POWER
        );
    }

    /**
     * Applies voltage compensation and clamps the final
     * motor command to the FTC SDK power range.
     */
    private double calculateCompensatedPower(
            double requestedPower
    ) {
        if (requestedPower == 0.0) {
            return 0.0;
        }

        final double compensatedPower =
                requestedPower
                        * voltageCompensation;

        return clamp(
                compensatedPower,
                -1.0,
                1.0
        );
    }

    private boolean shouldWritePower(
            double newPower,
            double lastPower
    ) {
        /*
         * Always write an exact zero when stopping.
         */
        if (newPower == 0.0
                && lastPower != 0.0) {

            return true;
        }

        if (newPower != 0.0) {
            return Math.abs(
                    newPower - lastPower
            ) > WRITE_TOLERANCE;
        }

        return false;
    }

    public void telemetry() {
        if (telemetry == null) {
            return;
        }

        telemetry.addData(
                "Intake",
                getIntakeState()
        );

        telemetry.addData(
                "Intake Requested",
                intakePower
        );

        telemetry.addData(
                "Intake Applied",
                appliedIntakePower
        );

        telemetry.addData(
                "Transfer",
                getTransferState()
        );

        telemetry.addData(
                "Transfer Requested",
                transferPower
        );

        telemetry.addData(
                "Transfer Applied",
                appliedTransferPower
        );

        telemetry.addData(
                "Intake Voltage Comp",
                voltageCompensation
        );
    }

    private static double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}