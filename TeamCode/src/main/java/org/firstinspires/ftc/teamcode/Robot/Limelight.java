package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private LLResult latestResult;
    private double currentTx = 0.0;
    private boolean targetVisible = false;
    private int detectedTagId = -1;
    private boolean freshFrameThisLoop = false;

    // Expected tx for the current shot / pose
    private double targetTx = 0.0;

    // Hold last valid reading briefly to avoid single-frame flicker
    private long lastValidFrameMs = 0;
    private static final long VISION_HOLD_MS = 120;

    // Tag filtering like your 2nd version
    private final List<Integer> allowedTagIds = new ArrayList<>();

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, HamiltonParams.HW_LIMELIGHT);

        // Start on pipeline 4
        limelight.pipelineSwitch(4);
        // limelight.start();
        limelight.stop();
    }

    public void pollVision() {
        LLResult result = limelight.getLatestResult();
        freshFrameThisLoop = (result != latestResult);
        latestResult = result;

        long nowMs = System.currentTimeMillis();

        if (latestResult != null && latestResult.isValid()) {
            LLResultTypes.FiducialResult bestTag = null;
            double bestArea = -1.0;

            List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = (int) tag.getFiducialId();

                if (allowedTagIds.isEmpty() || allowedTagIds.contains(id)) {
                    if (tag.getTargetArea() > bestArea) {
                        bestArea = tag.getTargetArea();
                        bestTag = tag;
                    }
                }
            }

            if (bestTag != null) {
                targetVisible = true;
                currentTx = bestTag.getTargetXDegrees();
                detectedTagId = (int) bestTag.getFiducialId();
                lastValidFrameMs = nowMs;
            } else {
                // No allowed tag found, briefly hold visibility
                targetVisible = (nowMs - lastValidFrameMs) <= VISION_HOLD_MS;
                if (!targetVisible) {
                    detectedTagId = -1;
                }
            }
        } else {
            // Preserve visibility very briefly to survive a dropped frame
            targetVisible = (nowMs - lastValidFrameMs) <= VISION_HOLD_MS;

            if (!targetVisible) {
                detectedTagId = -1;
            }
        }
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public double getTx() {
        return currentTx;
    }

    public double getCorrectedTx() {
        return currentTx - HamiltonParams.LIMELIGHT_MOUNT_OFFSET_DEG;
    }

    // Blue / Red filtering like the 2nd class
    public void setTargetBlue() {
        setAllowedTags(20);
        limelight.pipelineSwitch(4);
    }

    public void setTargetRed() {
        setAllowedTags(24);
        limelight.pipelineSwitch(4);
    }

    public void setTargetAngle(double targetTx) {
        this.targetTx = targetTx;
    }

    public double getTargetAngle() {
        return targetTx;
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

    public double getHeadingBiasObservationDeg() {
        return getCorrectedTx() - targetTx;
    }

    // Keep for compatibility
    public double getLastError() {
        return getHeadingBiasObservationDeg();
    }

    public String getAimProfileName() {
        return "Limelight";
    }

    public void updateControl() {
        // No-op; used only as heading observation source
    }

    public double getTurnPower() {
        return 0.0;
    }

    public void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("LL Visible", targetVisible);
        telemetry.addData("LL tx Raw", "%.2f", currentTx);
        telemetry.addData("LL tx Corrected", "%.2f", getCorrectedTx());
        telemetry.addData("LL Mount Offset", "%.2f", HamiltonParams.LIMELIGHT_MOUNT_OFFSET_DEG);
        telemetry.addData("LL Target tx", "%.2f", targetTx);
        telemetry.addData("LL Heading Bias Obs", "%.2f", getHeadingBiasObservationDeg());
        telemetry.addData("LL Fresh", freshFrameThisLoop);
        telemetry.addData("LL Tag ID", detectedTagId);
    }

    private void setAllowedTags(Integer... tags) {
        allowedTagIds.clear();
        allowedTagIds.addAll(Arrays.asList(tags));
    }
    public long getLastTargetAcquiredMs() {
        return lastValidFrameMs;
    }
}