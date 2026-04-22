package org.firstinspires.ftc.teamcode.Helpers;

import com.pedropathing.geometry.Pose;

public final class FieldMirror {

    private FieldMirror() {}

    public static Pose pose(Alliance alliance, double x, double y, double headingDeg) {
        if (alliance == Alliance.BLUE) {
            return new Pose(x, y, Math.toRadians(headingDeg));
        } else {
            return new Pose(x, -y, Math.toRadians(-headingDeg));
        }
    }

    public static double headingRad(Alliance alliance, double headingDeg) {
        return Math.toRadians(alliance == Alliance.BLUE ? headingDeg : -headingDeg);
    }
}