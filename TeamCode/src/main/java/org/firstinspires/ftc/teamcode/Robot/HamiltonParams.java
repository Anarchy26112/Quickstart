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
    public static int POSITIONS_PER_TURN = 6;
    public static int MAX_POSITION = 23;
    public static int MIN_POSITION = 6;
    public static double SERVO_SCALE = 30.0;
    public static int OPTIMAL_DISTANCE = 15;
    public static double OFFSET = 0;

    // ========== COLOR SENSOR ==========
    // Distance threshold for reliable color detection (in mm)
    public static final double MAX_DETECTION_DISTANCE_MM = 50.0;
    public static final float MIN_BRIGHTNESS = 0.1f;

    // ========== HARDWARE DEVICE NAMES ==========
    public static final String HW_INTAKE = "intake";
    public static final String HW_SHOOTER = "Shooter";
    public static final String HW_PUSHER = "pusher";
    public static final String HW_SPINDEX = "spinDex";
    public static final String HW_COLOR_SENSOR_LEFT = "ballColorSensorL";
    public static final String HW_COLOR_SENSOR_RIGHT = "ballColorSensorR";
}

