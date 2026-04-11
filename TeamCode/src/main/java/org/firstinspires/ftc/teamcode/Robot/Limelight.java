package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Limelight {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private LLResult latestResult;

    private double currentTx = 0.0;
    private double currentTargetArea = 0.0;

    private boolean targetVisible = false;
    private boolean controlTargetVisible = false;
    private int detectedTagId = -1;
    private boolean freshFrameThisLoop = false;

    // Normal shot-aim target tx
    private double targetTx = 0.0;

    // Tag-centering target tx
    private double tagCenterTargetTx = 0.0;

    // true => use image-space tag centering observation
    private boolean usePureTagCenteringObservation = false;

    private long lastValidFrameMs = 0;

    private static final long VISION_HOLD_MS = 120;
    private static final long CONTROL_FRAME_MAX_AGE_MS = 55;

    private static final double MIN_ACCEPTED_TAG_AREA = 0.08;
    private static final double MAX_ACCEPTED_ABS_TX_DEG = 27.0;

    // Hot-path optimization: replace ArrayList<Integer>.contains(id)
    // with primitive checks while preserving behavior.
    private boolean allowAnyTag = true;
    private int allowedTagA = -1;
    private int allowedTagB = -1;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, HamiltonParams.HW_LIMELIGHT);

        limelight.pipelineSwitch(4);
        limelight.stop();
    }

    public void pollVision(long nowMs) {
        LLResult result = limelight.getLatestResult();
        freshFrameThisLoop = (result != latestResult);
        latestResult = result;

        LLResultTypes.FiducialResult bestTag = null;

        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();

            for (int i = 0, n = tags.size(); i < n; i++) {
                LLResultTypes.FiducialResult tag = tags.get(i);

                int id = (int) tag.getFiducialId();
                double tx = tag.getTargetXDegrees();
                double ta = tag.getTargetArea();

                boolean idAllowed = isAllowedTag(id);
                boolean txOk = Math.abs(tx) <= MAX_ACCEPTED_ABS_TX_DEG;
                boolean areaOk = ta >= MIN_ACCEPTED_TAG_AREA;

                if (!idAllowed || !txOk || !areaOk) continue;

                bestTag = tag;
                break;
            }
        }

        if (bestTag != null) {
            targetVisible = true;
            controlTargetVisible = true;
            currentTx = bestTag.getTargetXDegrees();
            currentTargetArea = bestTag.getTargetArea();
            detectedTagId = (int) bestTag.getFiducialId();
            lastValidFrameMs = nowMs;
            return;
        }

        long frameAgeMs = nowMs - lastValidFrameMs;

        targetVisible = frameAgeMs <= VISION_HOLD_MS;
        controlTargetVisible = frameAgeMs <= CONTROL_FRAME_MAX_AGE_MS;

        if (!controlTargetVisible) {
            currentTx = 0.0;
        }

        if (!targetVisible) {
            detectedTagId = -1;
            currentTargetArea = 0.0;
        }
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isControlTargetVisible() {
        return controlTargetVisible;
    }

    public double getTx() {
        return currentTx;
    }

    public double getCorrectedTx() {
        return currentTx - HamiltonParams.LIMELIGHT_MOUNT_OFFSET_DEG;
    }

    public double getTargetArea() {
        return currentTargetArea;
    }

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

    public void setTagCenterTargetAngle(double targetTx) {
        this.tagCenterTargetTx = targetTx;
    }

    public double getTagCenterTargetAngle() {
        return tagCenterTargetTx;
    }

    public void setUsePureTagCenteringObservation(boolean enabled) {
        this.usePureTagCenteringObservation = enabled;
    }

    public boolean isUsingPureTagCenteringObservation() {
        return usePureTagCenteringObservation;
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

    public double getAimHeadingBiasObservationDeg() {
        return getCorrectedTx() - targetTx;
    }

    public double getTagCenteringErrorDeg() {
        return getCorrectedTx() - tagCenterTargetTx;
    }

    public double getHeadingBiasObservationDeg() {
        if (usePureTagCenteringObservation) {
            return getTagCenteringErrorDeg();
        }
        return getAimHeadingBiasObservationDeg();
    }

    public double getLastError() {
        return getHeadingBiasObservationDeg();
    }

    public String getAimProfileName() {
        return usePureTagCenteringObservation ? "LimelightTagCentering" : "LimelightAim";
    }

    public void updateControl() {
        // No-op
    }

    public double getTurnPower() {
        return 0.0;
    }

    public long getControlFrameAgeMs() {
        return System.currentTimeMillis() - lastValidFrameMs;
    }
    public long getLastTargetAcquiredMs() {
        return lastValidFrameMs;
    }
    public void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("LL Visible", targetVisible);
        telemetry.addData("LL Control Visible", controlTargetVisible);
        telemetry.addData("LL tx Raw", "%.2f", currentTx);
        telemetry.addData("LL tx Corrected", "%.2f", getCorrectedTx());
        telemetry.addData("LL Target Area", "%.3f", currentTargetArea);
        telemetry.addData("LL Mount Offset", "%.2f", HamiltonParams.LIMELIGHT_MOUNT_OFFSET_DEG);

        telemetry.addData("LL Aim Target tx", "%.2f", targetTx);
        telemetry.addData("LL Center Target tx", "%.2f", tagCenterTargetTx);
        telemetry.addData("LL Center Mode", usePureTagCenteringObservation);

        telemetry.addData("LL Aim Bias Obs", "%.2f", getAimHeadingBiasObservationDeg());
        telemetry.addData("LL Tag Centering Err", "%.2f", getTagCenteringErrorDeg());
        telemetry.addData("LL Active Bias Obs", "%.2f", getHeadingBiasObservationDeg());

        telemetry.addData("LL Fresh", freshFrameThisLoop);
        telemetry.addData("LL Tag ID", detectedTagId);
        telemetry.addData("LL Control Frame Age Ms", getControlFrameAgeMs());
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    private boolean isAllowedTag(int id) {
        return allowAnyTag || id == allowedTagA || id == allowedTagB;
    }

    private void setAllowedTags(int... ids) {
        if (ids == null || ids.length == 0) {
            allowAnyTag = true;
            allowedTagA = -1;
            allowedTagB = -1;
            return;
        }

        allowAnyTag = false;
        allowedTagA = ids[0];
        allowedTagB = ids.length > 1 ? ids[1] : -1;
    }
}