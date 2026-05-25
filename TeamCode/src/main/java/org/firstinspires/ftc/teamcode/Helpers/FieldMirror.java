package org.firstinspires.ftc.teamcode.Helpers;

import com.pedropathing.geometry.Pose;

public final class FieldMirror {

    private FieldMirror() {}

    public static Pose pose(Alliance alliance, double x, double y, double headingDeg) {
        Pose bluePose = new Pose(x, y, Math.toRadians(headingDeg));

        if (alliance == Alliance.BLUE) {
            return bluePose;
        }

        return bluePose.mirror();
    }
}