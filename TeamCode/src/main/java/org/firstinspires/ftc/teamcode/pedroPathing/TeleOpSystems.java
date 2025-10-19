package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class defines a subsystem for driving a robot with a Mecanum drive system.
 * It handles the initialization of motors and provides a method to control the robot's movement.
 */
public class TeleOpSystems {
    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;
    private final DcMotor intake;
    private final DcMotor outtake1;
    private final DcMotor outtake2;
    private final Servo spindex;
    private final ColorRangeSensor ballColorSensor;

    private final Telemetry telemetry;
    private String [] slots = {"green", "purple", "purple"};
    private int currentPosition = 3;
    private int currentTurn = 2;

    /**
     * Constructor for the DriveSubsystem.
     * Initializes the motors and sets their directions based on the robot's configuration.
     *
     * @param hardwareMap The hardware map to access the robot's hardware
     * @param opModeTelemetry The telemetry object for sending data to the driver station.
     */
    public TeleOpSystems(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        // Initialize the hardware
        // The strings here must correspond to the names of the motors in your robot config
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        intake = hardwareMap.dcMotor.get("intake");
        outtake1 = hardwareMap.dcMotor.get("outtake1");
        outtake2 = hardwareMap.dcMotor.get("outtake2");
        spindex = hardwareMap.servo.get("spindex");
        ballColorSensor = (ColorRangeSensor) hardwareMap.colorSensor.get("colorSensor");

        // Set motor directions
        // You may need to adjust these for your particular robot
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        telemetry = opModeTelemetry;
    }

    /**
     * Drives the robot using the Mecanum drive system.
     * Combines axial, lateral, and yaw inputs to calculate the power for each wheel.
     *
     * @param axial   The forward/backward movement input (negative for forward)
     * @param lateral The left/right strafe input
     * @param yaw     The rotation input (positive for clockwise rotation)
     */
    public void drive(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
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
    private void launch(double power){
        outtake1.setPower(power);
        outtake2.setPower(power);
        telemetry.addData("Outtake", "%4.2f, %4.2f", power, power);
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
            spindex.setPosition(target2/30);
            telemetry.addData("Current Position", targetPosition);//
            telemetry.update();
            currentPosition = targetPosition;
        }
        else if(target2>23){
            spindex.setPosition(target1/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;
            currentTurn -= 1;
        }
        else{
            if(distance1>distance2){
                spindex.setPosition(target2/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
            }
            else if(distance2>distance1){
                spindex.setPosition(target1/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
                currentTurn -= 1;
            }
            else{
                if(Math.abs(distance1-15)>Math.abs(distance2-15)){
                    spindex.setPosition(target2/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                }
                else if (Math.abs(distance1-15)<Math.abs(distance2-15)) {
                    spindex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn -=1;
                }
                else{
                    spindex.setPosition(target1/30);
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
            spindex.setPosition(target2/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;
            currentTurn +=1;
        }
        else if(target2>23){
            spindex.setPosition(target1/30);
            telemetry.addData("Current Position", targetPosition);
            telemetry.update();
            currentPosition = targetPosition;

        }
        else{
            if(distance1>distance2){
                spindex.setPosition(target2/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;
                currentTurn += 1;
            }
            else if(distance2>distance1){
                spindex.setPosition(target1/30);
                telemetry.addData("Current Position", targetPosition);
                telemetry.update();
                currentPosition = targetPosition;

            }
            else{
                if(Math.abs(distance1-15)>Math.abs(distance2-15)){
                    spindex.setPosition(target2/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn += 1;
                }
                else if (Math.abs(distance1-15)<Math.abs(distance2-15)) {
                    spindex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                }
                else{
                    spindex.setPosition(target1/30);
                    telemetry.addData("Current Position", targetPosition);
                    telemetry.update();
                    currentPosition = targetPosition;
                    currentTurn += 1;
                }
            }
        }
    }
}

