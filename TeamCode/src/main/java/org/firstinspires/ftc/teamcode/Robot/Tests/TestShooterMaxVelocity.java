package org.firstinspires.ftc.teamcode.Robot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

@TeleOp(name = "Test Shooter Max Velocity", group = "Tests")
public class TestShooterMaxVelocity extends LinearOpMode {

    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;

    private double maxRightVelocity = 0.0;
    private double maxLeftVelocity = 0.0;
    private double maxAverageVelocity = 0.0;

    private double testPower = 0.0;

    @Override
    public void runOpMode() {

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        configureMotor(rightShooter, DcMotor.Direction.FORWARD);
        configureMotor(leftShooter, DcMotor.Direction.REVERSE);

        telemetry.addLine("Shooter Max Velocity Test");
        telemetry.addLine("Put robot safely on blocks.");
        telemetry.addLine("Press START, then:");
        telemetry.addLine("A = run full power");
        telemetry.addLine("B = stop");
        telemetry.addLine("X = reset max velocity");
        telemetry.addLine("D-pad up/down = adjust test power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                testPower = 1.0;
            }

            if (gamepad1.b) {
                testPower = 0.0;
            }

            if (gamepad1.dpad_up) {
                testPower += 0.01;
                sleep(120);
            }

            if (gamepad1.dpad_down) {
                testPower -= 0.01;
                sleep(120);
            }

            if (gamepad1.x) {
                maxRightVelocity = 0.0;
                maxLeftVelocity = 0.0;
                maxAverageVelocity = 0.0;
                sleep(250);
            }

            if (testPower > 1.0) testPower = 1.0;
            if (testPower < 0.0) testPower = 0.0;

            rightShooter.setPower(testPower);
            leftShooter.setPower(testPower);

            double rightVelocity = abs(rightShooter.getVelocity());
            double leftVelocity = abs(leftShooter.getVelocity());
            double averageVelocity = 0.5 * (rightVelocity + leftVelocity);

            if (rightVelocity > maxRightVelocity) {
                maxRightVelocity = rightVelocity;
            }

            if (leftVelocity > maxLeftVelocity) {
                maxLeftVelocity = leftVelocity;
            }

            if (averageVelocity > maxAverageVelocity) {
                maxAverageVelocity = averageVelocity;
            }

            double estimatedF = maxAverageVelocity > 0.0
                    ? 32767.0 / maxAverageVelocity
                    : 0.0;

            telemetry.addData("Test Power", "%.2f", testPower);

            telemetry.addLine();
            telemetry.addData("Right Velocity", "%.1f ticks/sec", rightVelocity);
            telemetry.addData("Left Velocity", "%.1f ticks/sec", leftVelocity);
            telemetry.addData("Average Velocity", "%.1f ticks/sec", averageVelocity);

            telemetry.addLine();
            telemetry.addData("Max Right Velocity", "%.1f ticks/sec", maxRightVelocity);
            telemetry.addData("Max Left Velocity", "%.1f ticks/sec", maxLeftVelocity);
            telemetry.addData("Max Avg Velocity", "%.1f ticks/sec", maxAverageVelocity);

            telemetry.addLine();
            telemetry.addData("Estimated Built-in PIDF F", "%.2f", estimatedF);

            telemetry.addLine();
            telemetry.addLine("A = full power");
            telemetry.addLine("B = stop");
            telemetry.addLine("X = reset max");
            telemetry.addLine("D-pad = adjust power");

            telemetry.update();
        }

        rightShooter.setPower(0.0);
        leftShooter.setPower(0.0);
    }

    private void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Use RUN_WITHOUT_ENCODER here because we are measuring the raw
         * maximum physical shooter speed, not using built-in velocity PIDF yet.
         */
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(0.0);
    }

    private double abs(double value) {
        return value < 0.0 ? -value : value;
    }
}