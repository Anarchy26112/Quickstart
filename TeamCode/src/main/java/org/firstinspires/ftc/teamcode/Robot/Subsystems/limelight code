package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimelightHelpers;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
@Disabled
public class sensorlimelight3A_1_ extends LinearOpMode {

    private Limelight3A limelight;
    private NetworkTable limelightTable;

    @Override
    public void runOpMode() throws InterruptedException {

        limelightTable = NetworkTableInstance.getDefault().getTable("Limelight3A");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Bluety", result.BotPoseBlueTy());

                telemetry.addData("Botpose", result.getBotpose().toString());

                telemetry.addData("Distance (in)", getDistanceToTarget());
                telemetry.addData("Horizontal Offset", getOffsetAngle());

            } else {
                telemetry.addData("Limelight", "No valid data");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    // -----------------------------------------------------
    // Helper Method: Distance to target using vertical angle
    // -----------------------------------------------------
    public double getDistanceToTarget() {

        final double TARGET_HEIGHT_IN = 30.0;
        final double LL_HEIGHT_IN = 9.0;
        final double LL_MOUNT_ANGLE_DEG = 25.0; // Change to your physical mount angle

        double ty = LimelightHelpers.getTy("limelight"); // vertical offset

        double angleToGoal = LL_MOUNT_ANGLE_DEG + ty;

        double distance = (TARGET_HEIGHT_IN - LL_HEIGHT_IN) /
                Math.tan(Math.toRadians(angleToGoal));

        SmartDashboard.putNumber("Distance to Target (in)", distance);
        return distance;
    }

    // -----------------------------------------------------
    // Helper Method: Horizontal offset
    // -----------------------------------------------------
    public double getOffsetAngle() {
        return LimelightHelpers.getTx("limelight");
    }

    // -----------------------------------------------------
    // Returns AprilTag IDs
    // -----------------------------------------------------
    public double getAprilTagId() {
        return limelightTable.getEntry("tid").getDouble(-1);
    }
}
