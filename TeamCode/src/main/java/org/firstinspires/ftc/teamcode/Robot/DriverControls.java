package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

public class DriverControls {
    private Follower follower;
    private Telemetry telemetry;

    // Speed mode tracking
    private boolean slowMode = false;

    public DriverControls(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize Pedro Pathing follower (handles all drive motors internally)
        /*
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose());
        follower.update();
         */
    }

    /**
     * Start teleop drive mode
     * Call this in TeleOp start()
     */
    /*
    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }
    */

    /**
     * Update drivetrain with gamepad input
     * Call this every loop
     */
    /*
    public void update(Gamepad gamepad1) {
        // Update follower (required every loop)
        follower.update();

        // Toggle slow mode with right bumper
        // Normal/slow mode
        slowMode = !gamepad1.right_bumper;  // Fast mode

        // Set teleop drive (Pedro Pathing handles all the motor math)
        if (!slowMode) {
            // Full speed mode (100%)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true  // Robot-centric (change to false for field-centric)
            );
        } else {
            // Normal speed mode (55%)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * NORMAL_SPEED,
                    -gamepad1.left_stick_x * NORMAL_SPEED,
                    -gamepad1.right_stick_x * NORMAL_SPEED,
                    true  // Robot-centric (change to false for field-centric)
            );
        }
    }
    */


    public void updateTelemetry(Gamepad gamepad1) {
        if (!slowMode) {
            telemetry.addData("Drive Mode", "Full Speed (100%)");
        } else {
            telemetry.addData("Drive Mode", "Normal Speed (55%)");
        }

        telemetry.addData("X", String.format(Locale.US, "%.1f", follower.getPose().getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.1f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format(Locale.US, "%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Velocity", String.format(Locale.US, "%.2f", follower.getVelocity().getMagnitude()));
    }

    // Get follower for autonomous use
    public Follower getFollower() {
        return follower;
    }
}