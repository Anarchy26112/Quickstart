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
    public static double Kp_TURN = 0.009;
    public static double Kd_TURN = 0.0006;
    public static double kS_VOLTAGE_COMP = 0.053;
    public static double HEADING_kP = 0.27;
    // public static double HEADING_kP = 0.5;
    public static final double MAX_AUTO_TURN = 0.35;

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
