package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpinDex {
    public Servo spin_dex;
    public ColorRangeSensor ballColorSensor;
    private String [] slots = {"green", "purple", "purple"};
    private int currentPosition = 3;
    private int currentTurn = 2;
    public SpinDex(HardwareMap hardwareMap) {
        spin_dex = hardwareMap.get(Servo.class, "Spin_Dex");
    }
    private void sortBalls(int currentPos, int targetPosition, int currentRot){
        if(currentPos < targetPosition){
            sortBallsForward(currentPos, targetPosition, currentRot);
        }
        else if (targetPosition > currentPos){
            sortBallsBackward(currentPos, targetPosition, currentRot);
        }
        else{
            //This is blank because I want this to do nothing
        }
    }
    private void sortBallsForward(int currentPos, int targetPosition, int currentRot){ //Someone please make a better name
        int currentValue = currentRot*6+currentPos;
        int target1 = (currentRot-1)*6+targetPosition;
        int target2 = currentRot*6+targetPosition;
        int distance1 = Math.abs(target1-currentValue);
        int distance2 = Math.abs(target2-currentValue);
        if(target1<6){
            spin_dex.setPosition(target2/30);
            telemetry.addData("Current Position", targetPosition);//
            telemetry.update();
            currentPosition = targetPosition;
        }
        else if(target2>23){
            spin_dex.setPosition(target1/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;
            currentTurn -= 1;
        }
        else{
            if(distance1>distance2){
                spin_dex.setPosition(target2/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
            }
            else if(distance2>distance1){
                spin_dex.setPosition(target1/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
                currentTurn -= 1;
            }
            else{
                if(Math.abs(distance1-15)>Math.abs(distance2-15)){
                    spin_dex.setPosition(target2/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                }
                else if (Math.abs(distance1-15)<Math.abs(distance2-15)) {
                    spin_dex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn -=1;
                }
                else{
                    spin_dex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn -=1;
                }
            }
        }
    }
    private void sortBallsBackward(int currentPos, int targetPosition, int currentRot){ //Look at line 145
        int currentValue = currentRot*6+currentPos;
        int target1 = currentRot*6+targetPosition;
        int target2 = (currentRot+1)*6+targetPosition;
        int distance1 = Math.abs(target1-currentValue);
        int distance2 = Math.abs(target2-currentValue);
        if(target1<6){
            spin_dex.setPosition(target2/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;
            currentTurn +=1;
        }
        else if(target2>23){
            spin_dex.setPosition(target1/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;

        }
        else{
            if(distance1>distance2){
                spin_dex.setPosition(target2/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
                currentTurn += 1;
            }
            else if(distance2>distance1){
                spin_dex.setPosition(target1/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;

            }
            else{
                if(Math.abs(distance1-15)>Math.abs(distance2-15)){
                    spin_dex.setPosition(target2/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn += 1;
                }
                else if (Math.abs(distance1-15)<Math.abs(distance2-15)) {
                    spin_dex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                }
                else{
                    spin_dex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn += 1;
                }
            }
        }
    }
}


