package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_TRANSFER;

@TeleOp(name = "Intake Current Logger", group = "Test")
public class IntakeCurrentLogger extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx transfer;

    private double intakePowerCmd = 1.0;
    private double transferPowerCmd = 0.67;

    private double maxIntakeCurrent = 0.0;
    private double maxTransferCurrent = 0.0;

    @Override
    public void runOpMode() {
        intake = (DcMotorEx) hardwareMap.get(DcMotor.class, HW_INTAKE);
        transfer = (DcMotorEx) hardwareMap.get(DcMotor.class, HW_TRANSFER);

        // Match your subsystem setup
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Intake Current Logger ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("A = intake only forward");
        telemetry.addLine("B = transfer only forward");
        telemetry.addLine("X = both forward");
        telemetry.addLine("Y = both reverse");
        telemetry.addLine("Right bumper = stop all");
        telemetry.addLine("D-pad up/down = increase/decrease test power");
        telemetry.addLine("Left bumper = reset max current");
        telemetry.update();

        double testPower = 0.50;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastLeftBumper = false;

        waitForStart();

        while (opModeIsActive()) {

            // Edge detect for power adjustment
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean leftBumper = gamepad1.left_bumper;

            if (dpadUp && !lastDpadUp) {
                testPower = Math.min(1.0, testPower + 0.05);
            }
            if (dpadDown && !lastDpadDown) {
                testPower = Math.max(0.0, testPower - 0.05);
            }
            if (leftBumper && !lastLeftBumper) {
                maxIntakeCurrent = 0.0;
                maxTransferCurrent = 0.0;
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastLeftBumper = leftBumper;

            // Controls
            if (gamepad1.a) {
                intakePowerCmd = testPower;
                transferPowerCmd = 0.0;
            } else if (gamepad1.b) {
                intakePowerCmd = 0.0;
                transferPowerCmd = testPower;
            } else if (gamepad1.x) {
                intakePowerCmd = testPower;
                transferPowerCmd = testPower;
            } else if (gamepad1.y) {
                intakePowerCmd = -testPower;
                transferPowerCmd = -testPower;
            } else if (gamepad1.right_bumper) {
                intakePowerCmd = 0.0;
                transferPowerCmd = 0.0;
            }

            intake.setPower(intakePowerCmd);
            transfer.setPower(transferPowerCmd);

            double intakeCurrent = intake.getCurrent(CurrentUnit.AMPS);
            double transferCurrent = transfer.getCurrent(CurrentUnit.AMPS);

            maxIntakeCurrent = Math.max(maxIntakeCurrent, Math.abs(intakeCurrent));
            maxTransferCurrent = Math.max(maxTransferCurrent, Math.abs(transferCurrent));

            telemetry.addData("Test Power", "%.2f", testPower);

            telemetry.addLine(" ");
            telemetry.addLine("COMMANDS");
            telemetry.addData("Intake Power Cmd", "%.2f", intakePowerCmd);
            telemetry.addData("Transfer Power Cmd", "%.2f", transferPowerCmd);

            telemetry.addLine(" ");
            telemetry.addLine("CURRENT");
            telemetry.addData("Intake Current (A)", "%.3f", intakeCurrent);
            telemetry.addData("Transfer Current (A)", "%.3f", transferCurrent);

            telemetry.addLine(" ");
            telemetry.addLine("MAX OBSERVED");
            telemetry.addData("Max Intake Current (A)", "%.3f", maxIntakeCurrent);
            telemetry.addData("Max Transfer Current (A)", "%.3f", maxTransferCurrent);

            telemetry.addLine(" ");
            telemetry.addLine("Suggested test:");
            telemetry.addLine("Run with 0, 1, 2, 3 balls and record settled current.");
            telemetry.update();
        }

        intake.setPower(0);
        transfer.setPower(0);
    }
}