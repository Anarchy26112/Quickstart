package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

/**
 * Safe wrapper around a Limelight 3A running the three-path Python pipeline.
 *
 * Python output contract:
 *   llpython[0] = decision_valid      (0 or 1)
 *   llpython[1] = path number         (1..3, populated even when invalid)
 *   llpython[2] = smoothed path-1 score
 *   llpython[3] = smoothed path-2 score
 *   llpython[4] = smoothed path-3 score
 *   llpython[5] = stable confidence   (0..1)
 *   llpython[6] = raw confidence      (0..1)
 *   llpython[7] = pipeline FPS
 *
 * Do not gate Python output on LLResult.isValid(). A Python pipeline may return
 * a useful llpython array while its contour is empty. llpython[0] is the
 * authoritative decision-valid flag.
 *
 * This class intentionally catches hardware/runtime exceptions so autonomous
 * can continue with its configured fallback plan when the camera is unavailable.
 */
public class LimelightScanner {

    public static final int LANE_UNKNOWN = -1;
    public static final int LANE_A = 0;
    public static final int LANE_B = 1;
    public static final int LANE_C = 2;

    private static final int PATH_COUNT = 3;
    private static final int LLPYTHON_LENGTH = 8;

    private static final int IDX_VALID = 0;
    private static final int IDX_PATH = 1;
    private static final int IDX_SCORE_BASE = 2;
    private static final int IDX_STABLE_CONF = 5;
    private static final int IDX_RAW_CONF = 6;
    private static final int IDX_FPS = 7;

    /**
     * Results older than this are ignored. The old 120 ms limit was too tight
     * for a Python pipeline near 10 FPS because one frame interval is already
     * about 100 ms before transport and OpMode-loop timing are included.
     */
    private static final long MAX_RESULT_STALENESS_MS = 500L;

    /** Hold llrobot[0] high long enough for multiple camera frames to see it. */
    private static final long RESET_HOLD_NS = 250_000_000L;

    private Limelight3A limelight;
    private final int pipelineIndex;
    private final double pollRateHz;

    /**
     * pathToLane[pipelinePath - 1] = lane index, where A=0, B=1, C=2.
     */
    private int[] pathToLane = {LANE_A, LANE_B, LANE_C};

    private boolean available = false;
    private boolean running = false;

    // -------------------------------------------------------------------------
    // Reset-pulse state
    // -------------------------------------------------------------------------

    private boolean resetPulseActive = false;
    private long resetPulseEndNs = 0L;
    private long lastResetReleaseNs = 0L;

    // -------------------------------------------------------------------------
    // Latest confirmed reading
    // -------------------------------------------------------------------------

    private int latchedLane = LANE_UNKNOWN;
    private double latchedConfidence = 0.0;

    /** Scores indexed by lane: [A, B, C]. */
    private final double[] latchedLaneScores = new double[PATH_COUNT];

    /** Estimated Limelight capture time, not the robot time of a repeated poll. */
    private long latchedCaptureNs = 0L;

    // -------------------------------------------------------------------------
    // Diagnostics from the latest accepted Python-output frame
    // -------------------------------------------------------------------------

    private int lastRawPath = -1;
    private boolean lastRawValid = false;
    private double lastStableConfidence = 0.0;
    private double lastRawConfidence = 0.0;
    private double lastPipelineFps = 0.0;

    /** Scores indexed by pipeline path: [P1, P2, P3]. */
    private final double[] lastRawPathScores = new double[PATH_COUNT];

    private long lastPollNs = 0L;
    private long lastFreshFrameNs = 0L;
    private long lastResultStalenessMs = -1L;
    private int consecutiveEmptyPolls = 0;
    private String lastRejectReason = "NOT STARTED";

    public LimelightScanner(
            HardwareMap hardwareMap,
            String deviceName,
            int pipelineIndex,
            double pollRateHz
    ) {
        this.pipelineIndex = pipelineIndex;
        this.pollRateHz = pollRateHz;

        try {
            limelight = hardwareMap.get(Limelight3A.class, deviceName);
            available = limelight != null;
            lastRejectReason = available ? "READY" : "NOT IN CONFIG";
        } catch (Exception ignored) {
            limelight = null;
            available = false;
            lastRejectReason = "NOT IN CONFIG";
        }
    }

    public boolean isAvailable() {
        return available;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * Installs a path-to-lane map only when it is a valid permutation of
     * {0, 1, 2}. Invalid maps are ignored so a bad alliance override cannot
     * silently create an impossible lane.
     */
    public void setPathToLaneMap(int[] map) {
        if (map == null || map.length != PATH_COUNT) {
            return;
        }

        boolean[] seen = new boolean[PATH_COUNT];
        for (int lane : map) {
            if (lane < LANE_A || lane > LANE_C || seen[lane]) {
                return;
            }
            seen[lane] = true;
        }

        pathToLane = new int[]{map[0], map[1], map[2]};
    }

    public void start() {
        if (!available || running) {
            return;
        }

        try {
            limelight.setPollRateHz((int) Math.round(pollRateHz));
            limelight.pipelineSwitch(pipelineIndex);
            limelight.start();
            running = true;

            // Start from a known-low reset input. A later request can then
            // create a real rising edge in the Python pipeline.
            writePipelineReset(false);
            resetPulseActive = false;
            resetPulseEndNs = 0L;
            lastResetReleaseNs = 0L;
            clearLatch();
            lastRejectReason = "RUNNING";
        } catch (Exception ignored) {
            running = false;
            lastRejectReason = "START ERROR";
        }
    }

    public void stop() {
        if (!available) {
            running = false;
            return;
        }

        try {
            // Do not leave the Python reset input high between OpModes.
            writePipelineReset(false);
            limelight.stop();
        } catch (Exception ignored) {
            // Nothing useful to do on the way out.
        }

        resetPulseActive = false;
        resetPulseEndNs = 0L;
        running = false;
        lastRejectReason = "STOPPED";
    }

    /**
     * Starts a real reset pulse. Call this once at the start of a scan.
     *
     * Do not immediately call releasePipelineReset() from FarTripleBase. This
     * class holds llrobot[0] high for RESET_HOLD_NS and releases it from update().
     */
    public void requestPipelineReset() {
        clearLatch();

        if (!available || !running) {
            lastRejectReason = "RESET REQUEST WHILE STOPPED";
            return;
        }

        if (!writePipelineReset(true)) {
            resetPulseActive = false;
            resetPulseEndNs = 0L;
            lastRejectReason = "RESET WRITE ERROR";
            return;
        }

        resetPulseActive = true;
        resetPulseEndNs = System.nanoTime() + RESET_HOLD_NS;
        lastResetReleaseNs = 0L;
        lastRejectReason = "RESET HIGH";
    }

    /**
     * Manual/emergency reset release. Normal autonomous code should not call
     * this after requestPipelineReset(); update() releases the pulse itself.
     */
    public void releasePipelineReset() {
        final long nowNs = System.nanoTime();

        writePipelineReset(false);
        resetPulseActive = false;
        resetPulseEndNs = 0L;
        lastResetReleaseNs = nowNs;
        clearLatch();
        lastRejectReason = "RESET RELEASED MANUALLY";
    }

    private boolean writePipelineReset(boolean enabled) {
        if (!available || limelight == null) {
            return false;
        }

        try {
            limelight.updatePythonInputs(
                    new double[]{
                            enabled ? 1.0 : 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0
                    }
            );
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    /**
     * Services the reset pulse.
     *
     * @return true while update() must skip result processing.
     */
    private boolean servicePipelineReset(long nowNs) {
        if (!resetPulseActive) {
            return false;
        }

        if (nowNs < resetPulseEndNs) {
            lastRejectReason = "RESET HIGH";
            return true;
        }

        writePipelineReset(false);
        resetPulseActive = false;
        resetPulseEndNs = 0L;
        lastResetReleaseNs = nowNs;

        // Discard everything accumulated while reset was high. On later polls,
        // update() also rejects any result whose estimated capture time predates
        // this release time.
        clearLatch();
        lastRejectReason = "RESET RELEASED";
        return true;
    }

    public void clearLatch() {
        latchedLane = LANE_UNKNOWN;
        latchedConfidence = 0.0;
        latchedLaneScores[0] = 0.0;
        latchedLaneScores[1] = 0.0;
        latchedLaneScores[2] = 0.0;
        latchedCaptureNs = 0L;
    }

    /** Call once per OpMode loop. */
    public void update() {
        if (!available || !running) {
            return;
        }

        final long nowNs = System.nanoTime();
        lastPollNs = nowNs;

        // Hold the reset line across actual camera frames, then spend the
        // release iteration doing no reads. This prevents a reset-period result
        // from being latched on the same loop that reset goes low.
        if (servicePipelineReset(nowNs)) {
            return;
        }

        final LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (Exception ignored) {
            consecutiveEmptyPolls++;
            lastRejectReason = "RESULT READ ERROR";
            return;
        }

        if (result == null) {
            consecutiveEmptyPolls++;
            lastRejectReason = "NO RESULT";
            return;
        }

        long stalenessMs;
        try {
            stalenessMs = result.getStaleness();
        } catch (Exception ignored) {
            stalenessMs = 0L;
        }

        if (stalenessMs < 0L) {
            stalenessMs = 0L;
        }
        lastResultStalenessMs = stalenessMs;

        if (stalenessMs > MAX_RESULT_STALENESS_MS) {
            consecutiveEmptyPolls++;
            lastRejectReason = "STALE RESULT";
            return;
        }

        // Estimate when the camera produced this frame. Using this timestamp
        // means repeatedly polling the same LLResult does not keep a latch alive.
        final long stalenessNs;
        if (stalenessMs > Long.MAX_VALUE / 1_000_000L) {
            stalenessNs = Long.MAX_VALUE;
        } else {
            stalenessNs = stalenessMs * 1_000_000L;
        }
        final long captureNs = Math.max(0L, nowNs - stalenessNs);

        // After a reset, accept only frames captured after the reset input went
        // low. This blocks an old valid llpython array from being re-latched.
        if (lastResetReleaseNs != 0L && captureNs < lastResetReleaseNs) {
            consecutiveEmptyPolls++;
            lastRejectReason = "PRE-RESET FRAME";
            return;
        }

        final double[] python;
        try {
            python = result.getPythonOutput();
        } catch (Exception ignored) {
            consecutiveEmptyPolls++;
            lastRejectReason = "PYTHON READ ERROR";
            return;
        }

        if (python == null || python.length < LLPYTHON_LENGTH) {
            consecutiveEmptyPolls++;
            lastRejectReason = "SHORT PYTHON OUTPUT";
            return;
        }

        consecutiveEmptyPolls = 0;
        lastFreshFrameNs = captureNs;

        lastRawValid = finite(python[IDX_VALID]) && python[IDX_VALID] > 0.5;
        lastRawPath = finite(python[IDX_PATH])
                ? (int) Math.round(python[IDX_PATH])
                : -1;

        for (int pathIndex = 0; pathIndex < PATH_COUNT; pathIndex++) {
            lastRawPathScores[pathIndex] = safeNumber(
                    python[IDX_SCORE_BASE + pathIndex]
            );
        }

        lastStableConfidence = clamp01(safeNumber(python[IDX_STABLE_CONF]));
        lastRawConfidence = clamp01(safeNumber(python[IDX_RAW_CONF]));
        lastPipelineFps = Math.max(0.0, safeNumber(python[IDX_FPS]));

        if (!lastRawValid) {
            // Keep any prior latch until its capture timestamp ages out.
            lastRejectReason = "PIPELINE INVALID";
            return;
        }

        if (lastRawPath < 1 || lastRawPath > PATH_COUNT) {
            lastRejectReason = "BAD PATH";
            return;
        }

        final int lane = pathToLane[lastRawPath - 1];
        if (lane < LANE_A || lane > LANE_C) {
            lastRejectReason = "BAD LANE MAP";
            return;
        }

        latchedLane = lane;
        latchedConfidence = lastStableConfidence;

        // Convert pipeline-path scores into lane-ordered scores so telemetry
        // remains correct even when an alliance uses the reversed map.
        latchedLaneScores[0] = 0.0;
        latchedLaneScores[1] = 0.0;
        latchedLaneScores[2] = 0.0;
        for (int pathIndex = 0; pathIndex < PATH_COUNT; pathIndex++) {
            final int mappedLane = pathToLane[pathIndex];
            if (mappedLane >= LANE_A && mappedLane <= LANE_C) {
                latchedLaneScores[mappedLane] = lastRawPathScores[pathIndex];
            }
        }

        latchedCaptureNs = captureNs;
        lastRejectReason = "LATCHED";
    }

    /**
     * @return true when the confirmed lane is recent and confident enough.
     */
    public boolean hasUsableLane(double maxAgeSec, double minConfidence) {
        if (latchedLane == LANE_UNKNOWN || latchedCaptureNs == 0L) {
            return false;
        }

        if (!finite(maxAgeSec) || maxAgeSec < 0.0) {
            return false;
        }

        if (!finite(minConfidence)) {
            return false;
        }

        if (latchedConfidence < minConfidence) {
            return false;
        }

        return getLatchAgeSec() <= maxAgeSec;
    }

    public int getLane() {
        return latchedLane;
    }

    public double getConfidence() {
        return latchedConfidence;
    }

    /** Returns the latched score for lane A=0, B=1, or C=2. */
    public double getScore(int laneIndex) {
        if (laneIndex < LANE_A || laneIndex > LANE_C) {
            return 0.0;
        }
        return latchedLaneScores[laneIndex];
    }

    public double getLatchAgeSec() {
        if (latchedCaptureNs == 0L) {
            return Double.NaN;
        }
        return Math.max(
                0.0,
                (System.nanoTime() - latchedCaptureNs) / 1_000_000_000.0
        );
    }

    public double getSecondsSinceFreshFrame() {
        if (lastFreshFrameNs == 0L) {
            return Double.NaN;
        }
        return Math.max(
                0.0,
                (System.nanoTime() - lastFreshFrameNs) / 1_000_000_000.0
        );
    }

    public int getLastRawPath() {
        return lastRawPath;
    }

    public boolean wasLastFrameValid() {
        return lastRawValid;
    }

    public double getLastRawConfidence() {
        return lastRawConfidence;
    }

    public double getPipelineFps() {
        return lastPipelineFps;
    }

    public boolean isResetPulseActive() {
        return resetPulseActive;
    }

    public static String laneToString(int lane) {
        switch (lane) {
            case LANE_A:
                return "A";
            case LANE_B:
                return "B";
            case LANE_C:
                return "C";
            default:
                return "?";
        }
    }

    public String getStatusString() {
        if (!available) {
            return "NOT IN CONFIG";
        }
        if (!running) {
            return "STOPPED";
        }

        return String.format(
                Locale.US,
                "path=%d valid=%s lane=%s conf=%.2f age=%s fps=%.1f",
                lastRawPath,
                lastRawValid ? "Y" : "N",
                laneToString(latchedLane),
                latchedConfidence,
                formatSeconds(getLatchAgeSec()),
                lastPipelineFps
        );
    }

    /** Path-ordered current-frame scores, matching llpython[2..4]. */
    public String getRawScoreString() {
        return String.format(
                Locale.US,
                "P1=%.2f P2=%.2f P3=%.2f",
                lastRawPathScores[0],
                lastRawPathScores[1],
                lastRawPathScores[2]
        );
    }

    /** Lane-ordered scores from the most recently latched valid decision. */
    public String getScoreString() {
        return String.format(
                Locale.US,
                "A=%.2f B=%.2f C=%.2f",
                latchedLaneScores[LANE_A],
                latchedLaneScores[LANE_B],
                latchedLaneScores[LANE_C]
        );
    }

    public String getDebugString() {
        final String resetText = resetPulseActive ? "HIGH" : "LOW";
        final String staleText = lastResultStalenessMs >= 0L
                ? lastResultStalenessMs + "ms"
                : "--";

        return String.format(
                Locale.US,
                "reset=%s resultAge=%s freshAge=%s rawConf=%.2f empty=%d reason=%s",
                resetText,
                staleText,
                formatSeconds(getSecondsSinceFreshFrame()),
                lastRawConfidence,
                consecutiveEmptyPolls,
                lastRejectReason
        );
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double safeNumber(double value) {
        return finite(value) ? value : 0.0;
    }

    private static double clamp01(double value) {
        if (value < 0.0) {
            return 0.0;
        }
        if (value > 1.0) {
            return 1.0;
        }
        return value;
    }

    private static String formatSeconds(double seconds) {
        if (!finite(seconds)) {
            return "--";
        }
        return String.format(Locale.US, "%.2fs", seconds);
    }
}