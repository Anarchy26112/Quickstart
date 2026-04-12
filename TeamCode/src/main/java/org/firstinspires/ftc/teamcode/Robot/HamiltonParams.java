package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HamiltonParams {

    // --- Teleop drive ---
    public static double NORMAL_SPEED = 0.40;
    public static double MAX_AUTO_TURN = 0.68;

    // --- Intake heading assist ---
    public static double HEADING_kP = 0.52;

    // --- Goal position for odom auto aim (BLUE) ---
    public static double GOAL_X = 56.0;
    public static double GOAL_Y_BLUE = -132.0;
    public static double GOAL_Y_RED = 132.0;

    // --- Y-based aim heading offsets ---
    public static double Y_AIM_OFFSET_FAR_DEG = 7.0;
    public static double Y_AIM_OFFSET_MID_DEG = 5.0;
    public static double Y_AIM_OFFSET_NEAR_DEG = 3.5;

    // --- Profile switch threshold ---
    public static double FAST_AIM_Y_THRESHOLD = -36.0;

    // --- Fast / aggressive profile (robotY < -36) ---
    public static double FAST_KP_TURN = 0.0112;
    public static double FAST_KD_TURN = 0.00130;
    public static double FAST_kS_VOLTAGE_COMP = 0.04;
    public static double FAST_ERROR_DEADBAND_DEG = 1.3;

    // --- Precise / passive profile ---
    public static double PRECISE_KP_TURN = 0.0102;
    public static double PRECISE_KD_TURN = 0.00155;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.035;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.95;

    // --- Final aim checks ---
    public static double ODOM_AIM_DEADBAND_DEG = 0.85;

    // --- Limelight trim ---
    public static double LIMELIGHT_MOUNT_OFFSET_DEG = 0.0;
    public static double LIMELIGHT_TRIM_kP = 0.0165;
    public static double LIMELIGHT_TRIM_MAX = 0.085;
    public static double LIMELIGHT_TX_DEADBAND_DEG = 0.28;
    public static double VISION_ENABLE_ODOM_ERROR_DEG = 7.5;

    // --- Shoot ready ---
    public static double FAST_SHOOT_READY_HEADING_ERROR_DEG = 2.2;
    public static double PRECISE_SHOOT_READY_HEADING_ERROR_DEG = 1.5;
    public static double FAST_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC = 16.0;
    public static double PRECISE_SHOOT_READY_MAX_HEADING_RATE_DEG_PER_SEC = 7.5;
    public static long SHOOT_READY_SETTLE_MS = 0;

    // --- Confidence-weighted vision blend ---
    public static double VISION_CONFIDENCE_MIN_TO_BLEND = 0.58;
    public static double VISION_CONFIDENCE_MIN_TO_WRITEBACK = 0.96;
    public static double VISION_BLEND_ALPHA_MIN = 0.030;
    public static double VISION_BLEND_ALPHA_MAX = 0.10;
    public static double VISION_BLEND_DECAY_WHEN_LOST = 0.90;

    public static double VISION_CONFIDENCE_MIN_TAG_AREA = 0.16;
    public static double VISION_CONFIDENCE_GOOD_TAG_AREA = 0.50;

    // hardware names unchanged
    public static double SHOOTER_MAX_VELOCITY = 2797.2;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_LIMELIGHT = "Limelight";
    public static final String HW_GATE = "gateServo";

    // ========== GATE ==========
    public static final double GATE_BLOCK_POS = 0.20;
    public static final double GATE_OPEN_POS = 0.67;
}