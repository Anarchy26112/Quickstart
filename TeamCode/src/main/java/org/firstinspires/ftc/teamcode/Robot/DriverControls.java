package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.NORMAL_SPEED;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.Locale;

public class DriverControls {
    private Follower follower;
    private Telemetry telemetry;
    private boolean slowMode = false;

    public DriverControls(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        // Update follower (required every loop)
        follower.update();

        // Toggle slow mode with right bumper (Held down = Fast, Released = Slow/Normal)
        slowMode = !gamepad1.right_bumper;

        if (!slowMode) {
            // Full speed mode (100%)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        } else {
            // Normal speed mode (55%)
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * NORMAL_SPEED,
                    -gamepad1.left_stick_x * NORMAL_SPEED,
                    -gamepad1.right_stick_x * 0.40,
                    false
            );
        }
    }

    public void updateTelemetry() {
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

    public Follower getFollower() {
        return follower;
    }
}