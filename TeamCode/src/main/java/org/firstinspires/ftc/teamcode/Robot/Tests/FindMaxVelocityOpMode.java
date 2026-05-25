package org.firstinspires.ftc.teamcode.Robot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.HW_RIGHT_SHOOTER;

@Config
@TeleOp(name = "Tuner: Find Max Velocity", group = "Test")
public class FindMaxVelocityOpMode extends LinearOpMode {

    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;
    private LynxModule controlHub;

    private double maxObservedR = 0.0;
    private double maxObservedL = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Link standard telemetry to FTC Dashboard for easy visualization charting
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Setup manual bulk caching to ensure high frequency accurate readings
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent()) controlHub = hub;
        }

        rightShooter = hardwareMap.get(DcMotorEx.class, HW_RIGHT_SHOOTER);
        leftShooter = hardwareMap.get(DcMotorEx.class, HW_LEFT_SHOOTER);

        // Configure identically to your Shooter subsystem
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press START to run wheels at 100% power.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Clear bulk cache for accurate current loop updates
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.clearBulkCache();
            }

            // Command maximum un-throttled raw power
            rightShooter.setPower(1.0);
            leftShooter.setPower(1.0);

            // Fetch absolute raw encoder velocities (ticks per second)
            double currentR = Math.abs(rightShooter.getVelocity());
            double currentL = Math.abs(leftShooter.getVelocity());

            // Track absolute peak values achieved
            if (currentR > maxObservedR) maxObservedR = currentR;
            if (currentL > maxObservedL) maxObservedL = currentL;

            // Display current diagnostics
            telemetry.addLine("=== CURRENT STATUS ===");
            if (controlHub != null) {
                telemetry.addData("Battery Voltage", Math.round(controlHub.getInputVoltage(VoltageUnit.VOLTS) * 100.0) / 100.0);
            }
            telemetry.addData("Right Vel", Math.round(currentR * 10.0) / 10.0);
            telemetry.addData("Left Vel", Math.round(currentL * 10.0) / 10.0);

            telemetry.addLine("\n=== PEAK MAX VELOCITIES ===");
            telemetry.addData("Max Observed Right", Math.round(maxObservedR * 10.0) / 10.0);
            telemetry.addData("Max Observed Left", Math.round(maxObservedL * 10.0) / 10.0);

            double averageMax = (maxObservedR + maxObservedL) / 2.0;
            telemetry.addData("SUGGESTED MAX_VELOCITY", Math.round(averageMax * 10.0) / 10.0);
            telemetry.update();
        }

        // Safe spindown on stop
        rightShooter.setPower(0.0);
        leftShooter.setPower(0.0);
    }
}