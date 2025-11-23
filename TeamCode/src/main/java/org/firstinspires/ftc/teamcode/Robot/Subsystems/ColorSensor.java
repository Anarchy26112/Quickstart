package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot.HamiltonParams.*;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.graphics.Color;

public class ColorSensor {

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private NormalizedColorSensor leftSensor;
    private DistanceSensor leftDistance;
    private NormalizedColorSensor rightSensor;
    private DistanceSensor rightDistance;

    private final Telemetry telemetry;

    public ColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftSensor = hardwareMap.get(NormalizedColorSensor.class, HW_COLOR_SENSOR_LEFT);
        leftDistance = hardwareMap.get(DistanceSensor.class, HW_COLOR_SENSOR_LEFT);

        rightSensor = hardwareMap.get(NormalizedColorSensor.class, HW_COLOR_SENSOR_RIGHT);
        rightDistance = hardwareMap.get(DistanceSensor.class, HW_COLOR_SENSOR_RIGHT);
    }

    // Shared detection logic
    private DetectedColor detect(NormalizedColorSensor sensor, DistanceSensor distanceSensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        // Too far or too dark = invalid
        if (distanceSensor.getDistance(DistanceUnit.MM) > MAX_DETECTION_DISTANCE_MM ||
                rgba.alpha < MIN_BRIGHTNESS) {
            return DetectedColor.UNKNOWN;
        }

        // Normalize RGB (with floating point threshold)
        float total = rgba.red + rgba.green + rgba.blue;
        if (total < 0.001f) return DetectedColor.UNKNOWN;

        float r = rgba.red / total;
        float g = rgba.green / total;
        float b = rgba.blue / total;

        // Convert to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(r * 255), (int)(g * 255), (int)(b * 255), hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Reject desaturated (grayish) or too-dark colors
        if (saturation < 0.3f || value < 0.2f) {
            return DetectedColor.UNKNOWN;
        }

        // --- Color Detection ---
        // GREEN = roughly 80–160° hue
        if (hue >= 150 && hue <= 165) {
            return DetectedColor.GREEN;
        }

        // PURPLE = hue around 260–300° or near 0°
        if ((hue >= 217.5 && hue <= 232.5)) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }

    public DetectedColor detectColorL() {
        return detect(leftSensor, leftDistance);
    }

    public DetectedColor detectColorR() {
        return detect(rightSensor, rightDistance);
    }

    public String getDetailedColorInfoL() {
        NormalizedRGBA rgba = leftSensor.getNormalizedColors();

        // Calculate HSV
        float total = rgba.red + rgba.green + rgba.blue;
        float r = (total < 0.001f) ? 0 : rgba.red / total;
        float g = (total < 0.001f) ? 0 : rgba.green / total;
        float b = (total < 0.001f) ? 0 : rgba.blue / total;

        float[] hsv = new float[3];
        Color.RGBToHSV((int)(r * 255), (int)(g * 255), (int)(b * 255), hsv);

        return String.format("L Color=%s Dist=%.1f HSV=(%.1f°,%.2f,%.2f)",
                detectColorL(),
                leftDistance.getDistance(DistanceUnit.MM),
                hsv[0], hsv[1], hsv[2]);
    }

    public String getDetailedColorInfoR() {
        NormalizedRGBA rgba = rightSensor.getNormalizedColors();

        // Calculate HSV
        float total = rgba.red + rgba.green + rgba.blue;
        float r = (total < 0.001f) ? 0 : rgba.red / total;
        float g = (total < 0.001f) ? 0 : rgba.green / total;
        float b = (total < 0.001f) ? 0 : rgba.blue / total;

        float[] hsv = new float[3];
        Color.RGBToHSV((int)(r * 255), (int)(g * 255), (int)(b * 255), hsv);

        return String.format("R Color=%s Dist=%.1f HSV=(%.1f°,%.2f,%.2f)",
                detectColorR(),
                rightDistance.getDistance(DistanceUnit.MM),
                hsv[0], hsv[1], hsv[2]);
    }
}