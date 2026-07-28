package org.firstinspires.ftc.teamcode.Robot.Tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Connection Check", group = "Test")
public class LimelightConnectionCheck extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight.getStatus() != null) {
                telemetry.addData("Limelight Connection", "CONNECTED");
            } else {
                telemetry.addData("Limelight Connection", "DISCONNECTED / OFF");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
