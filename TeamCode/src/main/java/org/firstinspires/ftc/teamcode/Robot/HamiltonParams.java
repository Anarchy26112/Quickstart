package org.firstinspires.ftc.teamcode.Robot;

public class HamiltonParams {

    // ========== DRIVETRAIN ==========
    public static double NORMAL_SPEED = 0.35;
    public static double FULL_SPEED = 1.0;

    // ========== INTAKE ==========
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double SPIT_POWER = 0.55;

    // ========== GATE ==========
    public static final double GATE_BLOCK_POS = 0.20;
    public static final double GATE_OPEN_POS  = 0.67;

    // ========== SHOOTER ==========
    public static double SHOOTER_MAX_VELOCITY = 2797.2;
    public static double HIGH_VELOCITY_THRESHOLD = 2235.0;
    public static double LOW_VELOCITY_THRESHOLD  = 1815.0;

    // ========== COLOR SENSOR ==========
    public static final double MAX_DETECTION_DISTANCE_MM = 45.0;
    public static final float MIN_BRIGHTNESS = 0.12f;

    // ========== LIMELIGHT PID CONTROL ==========
    public static double Kp_TURN = 0.018;
    public static double Kd_TURN = 0.002;
    public static double HEADING_kP = 0.33;
    public static double ODOM_AIM_DEADBAND_DEG = 1.0;
    public static final double MAX_AUTO_TURN = 0.5;

    // Separate feedforward values
    public static double FAST_kS_VOLTAGE_COMP    = 0.060;
    public static double PRECISE_kS_VOLTAGE_COMP = 0.038;

    // ========== LIMELIGHT PROFILE SWITCH ==========
    public static double FAST_AIM_Y_THRESHOLD = -36.0;

    public static double FAST_KP_TURN = 0.018;
    public static double FAST_KD_TURN = 0.002;
    public static double FAST_ERROR_DEADBAND_DEG = 1.0;

    public static double FAST_SETTLE_ENTER_DEADBAND_DEG = 1.5;
    public static double FAST_SETTLE_EXIT_DEADBAND_DEG  = 2.0;
    public static double FAST_SETTLE_ENTER_RATE_DPS = 10.0;
    public static double FAST_SETTLE_EXIT_RATE_DPS  = 15.0;

    public static double FAST_SHOOT_READY_ENTER_DEADBAND_DEG = 2.5;
    public static double FAST_SHOOT_READY_EXIT_DEADBAND_DEG  = 3.0;
    public static double FAST_SHOOT_READY_ENTER_RATE_DPS = 20.0;
    public static double FAST_SHOOT_READY_EXIT_RATE_DPS  = 25.0;

    public static double PRECISE_KP_TURN = 0.012;
    public static double PRECISE_KD_TURN = 0.003;
    public static double PRECISE_ERROR_DEADBAND_DEG = 0.3;

    public static double PRECISE_SETTLE_ENTER_DEADBAND_DEG = 0.5;
    public static double PRECISE_SETTLE_EXIT_DEADBAND_DEG  = 0.8;
    public static double PRECISE_SETTLE_ENTER_RATE_DPS = 2.0;
    public static double PRECISE_SETTLE_EXIT_RATE_DPS  = 4.0;

    public static double PRECISE_SHOOT_READY_ENTER_DEADBAND_DEG = 1.0;
    public static double PRECISE_SHOOT_READY_EXIT_DEADBAND_DEG  = 1.5;
    public static double PRECISE_SHOOT_READY_ENTER_RATE_DPS = 5.0;
    public static double PRECISE_SHOOT_READY_EXIT_RATE_DPS  = 8.0;


    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_TRANSFER = "transfer";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
    public static final String HW_LIMELIGHT = "Limelight";
    public static final String HW_GATE = "gateServo";
}
