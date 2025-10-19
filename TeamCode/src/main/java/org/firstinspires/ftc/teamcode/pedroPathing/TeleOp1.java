package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode contains code for driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * Also note that it is critical to set the correct rotation direction for each motor.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 */
@TeleOp(name = "TeleOp", group = "Linear OpMode")
public class TeleOp1 extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        TeleOpSystems teleOpSystems = new TeleOpSystems(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Any code before this point runs once ONCE when you press INIT
        waitForStart();

        // Any code after this point runs ONCE after you press START

        // Any code in this loop runs REPEATEDLY until the driver presses STOP
        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;// Left joystick to go forward & strafe, and right joystick to rotate.

            if(gamepad1.right_bumper) {
                teleOpSystems.drive(axial, lateral, yaw);
            }
            else{
                teleOpSystems.drive(axial*.55, lateral*.55, yaw*.55);

            }
            if(gamepad2.a && intakeActive=false){

            }

        }
        // Any code after the while loop will run ONCE after the driver presses STOP
    }
}