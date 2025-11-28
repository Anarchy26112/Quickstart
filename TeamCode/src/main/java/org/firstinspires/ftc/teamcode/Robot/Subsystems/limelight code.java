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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimelightHelpers;

import java.util.Arrays;

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
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();

                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("Bluety", result.BotPoseBlueTy());

                telemetry.addData("Botpose", botpose.toString());

                // --------------------------------------------------------
                // BARCODE RESULTS
                // --------------------------------------------------------
                for (LLResultTypes.BarcodeResult br : result.getBarcodeResults()) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // --------------------------------------------------------
                // CLASSIFIER RESULTS
                // --------------------------------------------------------
                for (LLResultTypes.ClassifierResult cr : result.getClassifierResults()) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f",
                            cr.getClassName(), cr.getConfidence());
                }

                // --------------------------------------------------------
                // DETECTOR RESULTS
                // --------------------------------------------------------
                for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f",
                            dr.getClassName(), dr.getTargetArea());
                }

                // --------------------------------------------------------
                // FIDUCIAL RESULTS
                // --------------------------------------------------------
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                            fr.getFiducialId(), fr.getFamily(),
                            fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // --------------------------------------------------------
                // COLOR RESULTS
                // --------------------------------------------------------
                for (LLResultTypes.ColorResult cr : result.getColorResults()) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f",
                            cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }

                // --------------------------------------------------------
                // YOUR FUNCTIONS (CALLED + TELEMETRY)
                // --------------------------------------------------------
                telemetry.addData("DistanceToTarget", getDistanceToTarget());
                telemetry.addData("OffsetAngle", getoffsetangle());
                telemetry.addData("AprilTagIDs", Arrays.toString(getAprilTagIds()));

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    // ======================================================================
    // RESTORED + FIXED FUNCTIONS
    // ======================================================================

    // -------------------------------
    // getDistanceToTarget() - FIXED
    // -------------------------------
    public double getDistanceToTarget() {

        final double TARGET_HEIGHT_INCHES = 30.0;
        final double LIMELIGHT_HEIGHT_INCHES = 9.0;
        final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 90.0;

        double targetOffsetAngle_Vertical = LimelightHelpers.getTy("limelight");
        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double distance = (TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES)
                / Math.tan(angleToGoalRadians);

        SmartDashboard.putNumber("Distance to Target (Inches)", distance);

        return distance;
    }

    // -------------------------------
    // getoffsetangle() - FIXED
    // -------------------------------
    public double getoffsetangle() {
        double getoffsetangle = LimelightHelpers.getTx("limelight");
        return getoffsetangle;
    }

    // -------------------------------
    // getAprilTagIds() - FIXED
    // -------------------------------
    public double[] getAprilTagIds() {
        return limelightTable.getEntry("tids").getDoubleArray(new double[]{-1});
    }
}
