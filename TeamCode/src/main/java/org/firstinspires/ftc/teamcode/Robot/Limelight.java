package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private LLResult latestResult;
    private double currentTx = 0.0;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private boolean freshFrameThisLoop = false;

    // Used by DriverControlsRed
    private double targetTx = 0.0;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, HamiltonParams.HW_LIMELIGHT);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void pollVision() {
        LLResult result = limelight.getLatestResult();
        freshFrameThisLoop = (result != latestResult);
        latestResult = result;

        if (latestResult != null && latestResult.isValid()) {
            targetVisible = true;
            currentTx = latestResult.getTx();

            // If you later want actual AprilTag IDs, wire it here using your SDK’s LLResult API.
            detectedTagId = -1;
        } else {
            targetVisible = false;
            currentTx = 0.0;
            detectedTagId = -1;
        }
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public double getTx() {
        return currentTx;
    }

    public void setTargetBlue() {
        limelight.pipelineSwitch(0);
    }

    public void setTargetRed() {
        limelight.pipelineSwitch(1);
    }

    public void setTargetAngle(double targetTx) {
        this.targetTx = targetTx;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    public boolean isFreshFrameThisLoop() {
        return freshFrameThisLoop;
    }

    public double getFilteredRate() {
        return 0.0;
    }

    public double getLastError() {
        return currentTx - targetTx;
    }

    public String getAimProfileName() {
        return "Limelight";
    }

    public void updateControl() {
        // No-op; turn output is computed on demand by getTurnPower().
    }

    public double getTurnPower() {
        if (!targetVisible) return 0.0;

        double error = currentTx - targetTx;
        if (Math.abs(error) < HamiltonParams.LIMELIGHT_TX_DEADBAND_DEG) {
            return 0.0;
        }

        double turn = HamiltonParams.LIMELIGHT_TRIM_kP * error;
        return clamp(turn, -HamiltonParams.LIMELIGHT_TRIM_MAX, HamiltonParams.LIMELIGHT_TRIM_MAX);
    }

    public void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("LL Visible", targetVisible);
        telemetry.addData("LL tx", "%.2f", currentTx);
        telemetry.addData("LL Target tx", "%.2f", targetTx);
        telemetry.addData("LL Err", "%.2f", currentTx - targetTx);
        telemetry.addData("LL Fresh", freshFrameThisLoop);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}