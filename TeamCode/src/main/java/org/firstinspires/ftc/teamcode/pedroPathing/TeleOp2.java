package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.HamiltonParams.intakeActive;
import static java.util.logging.Logger.global;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;//
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;
@Configurable
@TeleOp
public class TeleOp2 extends OpMode {

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    @Override
    public void init() {
        TeleOpSystems teleOpSystems = new TeleOpSystems(hardwareMap, telemetry);
        HamiltonParams params = new HamiltonParams();
        boolean intakeActive = false;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        TeleOpSystems teleOpSystems = new TeleOpSystems(hardwareMap, telemetry);
        //Call this once per loop
        follower.update();
        telemetryM.update();
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;// Left joystick to go forward & strafe, and right joystick to rotate.

        if(gamepad1.right_bumper) {
            teleOpSystems.drive(axial, lateral, yaw);
        }
        else{
            teleOpSystems.drive(axial*.55, lateral*.55, yaw*.55);

        }
        if(gamepad2.a && intakeActive==false){
            teleOpSystems.intake();
            intakeActive = true;
        }
        else if(gamepad2.a && intakeActive==true){
            teleOpSystems.resetIntake();
            intakeActive=false;
        }
        if(intakeActive){
            teleOpSystems.intake();
        }
        //Automated PathFollowing
    }
    public void runTeleop(){

    }
}
