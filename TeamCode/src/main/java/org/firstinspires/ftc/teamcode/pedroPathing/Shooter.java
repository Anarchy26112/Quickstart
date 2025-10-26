package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotor right_shooter;
    public DcMotor left_shooter;
    public Shooter(HardwareMap hardwareMap) {
        right_shooter = hardwareMap.dcMotor.get("Right_Shooter");
        left_shooter = hardwareMap.dcMotor.get("Left_Shooter");

        right_shooter.setDirection(DcMotor.Direction.REVERSE);
        left_shooter.setDirection(DcMotor.Direction.FORWARD);
    }
    private void launch(double power){
        right_shooter.setPower(power);
        left_shooter.setPower(power);
        telemetry.addData("Outtake", "%4.2f, %4.2f", power, power);
    }
    public void shortLaunch(){
        launch(0.05);
    }
    public void farLaunch(){
        launch(0.15);
    }
    public void eject(){
        launch(0.02);
    }
    public void stopLaunch(){
        launch(0);
    }

}
