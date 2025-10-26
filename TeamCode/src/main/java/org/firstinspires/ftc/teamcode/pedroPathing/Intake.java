package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("Intake");

        intake.setDirection(DcMotor.Direction.FORWARD);
    }
    public void intake(){
        intakePower(0.5);
    }
    public void spit(){
        intakePower(-0.5);
    }
    public void resetIntake(){
        intakePower(0);
    }
    private void intakePower(double power){
        intake.setPower(power);
        telemetry.addData("Intake", "%4.2f", power);
        telemetry.update();
    }

}
