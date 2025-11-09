package org.firstinspires.ftc.teamcode.Robot.Subsystems;
import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {

    // Enum for detected colors
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private NormalizedColorSensor colorSensorL;
    private DistanceSensor distanceSensorL;
    private NormalizedColorSensor colorSensorR;
    private DistanceSensor distanceSensorR;
    private final Telemetry telemetry;



    public ColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, HW_COLOR_SENSOR_LEFT);
        distanceSensorL = hardwareMap.get(DistanceSensor.class, HW_COLOR_SENSOR_LEFT);
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, HW_COLOR_SENSOR_RIGHT);
        distanceSensorR = hardwareMap.get(DistanceSensor.class, HW_COLOR_SENSOR_RIGHT);
    }

    public DetectedColor detectColorL() {
        NormalizedRGBA colorsL = colorSensorL.getNormalizedColors();

        // Check distance and brightness
        if (getDistanceMML() > MAX_DETECTION_DISTANCE_MM || colorsL.alpha < MIN_BRIGHTNESS) {
            return DetectedColor.UNKNOWN;
        }

        float redL = colorsL.red;
        float greenL = colorsL.green;
        float blue = colorsL.blue;

        // Find the dominant color
        float max = Math.max(redL, Math.max(greenL, blue));

        // GREEN: Green is dominant
        if (greenL == max && greenL > redL * 1.3f && greenL > blue * 1.3f) {
            return DetectedColor.GREEN;
        }

        // PURPLE: Red and blue are high, greenL is low
        if (blue > greenL * 1.3f && Math.abs(redL -blue)<=0.04f) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
    public DetectedColor detectColorR() {
        NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();

        // Check distance and brightness
        if (getDistanceMMR() > MAX_DETECTION_DISTANCE_MM || colorsR.alpha < MIN_BRIGHTNESS) {
            return DetectedColor.UNKNOWN;
        }

        float redR = colorsR.red;
        float greenR = colorsR.green;
        float blueR = colorsR.blue;

        // Find the dominant color
        float max = Math.max(redR, Math.max(greenR, blueR));

        // GREEN: Green is dominant
        if (greenR == max && greenR > redR * 1.3f && greenR > blueR * 1.3f) {
            return DetectedColor.GREEN;
        }

        // PURPLE: Red and blueR are high, greenR is low
        if (blueR > greenR * 1.3f && Math.abs(redR - blueR)<=0.04f) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
    public DetectedColor detectColor(DetectedColor colorL, DetectedColor colorR){
        if(colorL.equals(colorR)){
            return colorL;
        }
        else if(colorL.equals(DetectedColor.UNKNOWN) && !colorR.equals(DetectedColor.UNKNOWN)){
            return colorR;
        }
        else if(!colorL.equals(DetectedColor.UNKNOWN) && colorR.equals(DetectedColor.UNKNOWN)){
            return colorR;
        }
        else{
            return DetectedColor.UNKNOWN;
        }
    }

    public double getDistanceMML() {
        return distanceSensorL.getDistance(DistanceUnit.MM);
    }
    public double getDistanceMMR(){
        return distanceSensorR.getDistance(DistanceUnit.MM);
    }

    // Values (0.0 - 1.0)
    public NormalizedRGBA getNormalizedColorsL() {
        return colorSensorL.getNormalizedColors();
    }
    public NormalizedRGBA getNormalizedColorsR(){
        return colorSensorR.getNormalizedColors();
    }
    public String getDetailedColorInfoL() {
        DetectedColor color = detectColorL();
        double distance = getDistanceMML();
        NormalizedRGBA rgba = getNormalizedColorsL();

        return String.format("Color: %s | Distance: %.1fmm | RGB: (%2.4f, %2.4f, %2.4f)",
                color.name(), distance, rgba.red*1000, rgba.green*1000, rgba.blue*1000);
    }
    public String getDetailedColorInfoR() {
        DetectedColor color = detectColorR();
        double distance = getDistanceMMR();
        NormalizedRGBA rgba = getNormalizedColorsR();

        return String.format("Color: %s | Distance: %.1fmm | RGB: (%2.4f, %2.4f, %2.4f)",
                color.name(), distance, rgba.red*1000, rgba.green*1000, rgba.blue*1000);
    }
}