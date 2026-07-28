package org.firstinspires.ftc.teamcode.Robot.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "Limelight Python Diagnostic", group = "Test")
public class LimelightPythonDiagnostic extends OpMode {

    private static final int PIPELINE = 0;

    private Limelight3A limelight;

    private double previousResultTimestamp = Double.NaN;
    private long repeatedResultCount = 0;

    @Override
    public void init() {
        telemetry.setAutoClear(true);
        telemetry.setMsTransmissionInterval(50);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Restart the FTC polling connection cleanly.
        limelight.stop();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        // Clear old robot-to-Python inputs.
        limelight.updatePythonInputs(new double[8]);
    }

    @Override
    public void init_loop() {
        showLimelightData();
    }

    @Override
    public void loop() {
        showLimelightData();
    }

    private void showLimelightData() {
        telemetry.clearAll();

        telemetry.addData(
                "Running / connected",
                "%s / %s",
                limelight.isRunning(),
                limelight.isConnected()
        );

        LLStatus status = limelight.getStatus();

        if (status != null) {
            telemetry.addData("LL name", status.getName());
            telemetry.addData("LL FPS", "%.1f", status.getFps());
            telemetry.addData("LL CPU", "%.1f%%", status.getCpu());
            telemetry.addData("Status pipeline", status.getPipelineIndex());
            telemetry.addData("Pipeline type", status.getPipelineType());
        } else {
            telemetry.addData("LL status", "null");
        }

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addData("LLResult", "null");
            telemetry.update();
            return;
        }

        double timestamp = result.getTimestamp();

        if (!Double.isNaN(previousResultTimestamp)
                && timestamp == previousResultTimestamp) {
            repeatedResultCount++;
        } else {
            repeatedResultCount = 0;
            previousResultTimestamp = timestamp;
        }

        telemetry.addData("Result pipeline", result.getPipelineIndex());
        telemetry.addData("Result valid", result.isValid());
        telemetry.addData("Result timestamp", "%.3f", timestamp);
        telemetry.addData("Result timestamp repeats", repeatedResultCount);
        telemetry.addData("Result staleness", "%d ms", result.getStaleness());

        double[] python = result.getPythonOutput();

        if (python == null) {
            telemetry.addData("Python output", "null");
        } else {
            telemetry.addData("Python length", python.length);
            telemetry.addData("Python full array", Arrays.toString(python));

            if (python.length >= 7) {
                telemetry.addData(
                        "P1 / P2 / P3",
                        "%.5f / %.5f / %.5f",
                        python[2],
                        python[3],
                        python[4]
                );
                telemetry.addData("Python heartbeat", "%.0f", python[6]);
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}