package org.firstinspires.ftc.teamcode.Robot;

public class HamiltonParams {
//Sir Hamiltonian the Clanker LXVII
    // ========== DRIVETRAIN ==========
    public static double NORMAL_SPEED = 0.55;
    public static double FULL_SPEED = 1.0;

    // ========== INTAKE ==========
    public static double INTAKE_POWER = 0.75;
    public static double SPIT_POWER = -0.75;

    // ========== SHOOTER ==========
    public static double SHOOTER_HIGH_POWER = 1.0;
    public static double SHOOTER_LOW_POWER = 0.6;

    // ========== PUSHER ==========
    public static double PUSHER_RETRACTED_POS = 0.2;
    public static double PUSHER_EXTENDED_POS = 0.1;
    public static double PUSHER_PUSH_DURATION_MS = 300;
    public static double PUSHER_RETRACT_DELAY_MS = 100;

    // ========== SPINDEX ==========
    public static final int SPINDEX_MAX_POSITIONS = 6;     // number of indexed slots
    public static final double OFFSET = 0.0;
    public static final double SERVO_TURN_RANGE_DEGREES = 1800.0;
    public static final double SERVO_UNITS_PER_POSITION = 60.0 / SERVO_TURN_RANGE_DEGREES;

    // ========== COLOR SENSOR ==========
    // Distance threshold for reliable color detection (in mm)
    public static final double MAX_DETECTION_DISTANCE_MM = 50.0;
    public static final float MIN_BRIGHTNESS = 0.1f;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_RIGHT_SHOOTER = "rightShooter";
    public static final String HW_LEFT_SHOOTER = "leftShooter";
    public static final String HW_PUSHER = "pusher";
    public static final String HW_SPINDEX = "spinDex";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
}

