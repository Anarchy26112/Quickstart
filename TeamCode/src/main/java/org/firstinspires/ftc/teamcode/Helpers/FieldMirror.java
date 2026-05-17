package org.firstinspires.ftc.teamcode.Helpers;

import com.pedropathing.geometry.Pose;

public final class FieldMirror {

    private FieldMirror() {}

    // Pedro Pathing field midline between blue and red sides
    private static final double FIELD_MIDLINE_X = 70.75;

    public static Pose pose(Alliance alliance, double x, double y, double headingDeg) {
        if (alliance == Alliance.BLUE) {
            return new Pose(x, y, Math.toRadians(headingDeg));
        } else {
            double mirroredX = mirrorX(x);
            double mirroredHeadingDeg = mirrorHeadingDeg(headingDeg);

            return new Pose(
                    mirroredX,
                    y,
                    Math.toRadians(mirroredHeadingDeg)
            );
        }
    }

    public static double headingRad(Alliance alliance, double headingDeg) {
        if (alliance == Alliance.BLUE) {
            return Math.toRadians(headingDeg);
        } else {
            return Math.toRadians(mirrorHeadingDeg(headingDeg));
        }
    }

    private static double mirrorX(double x) {
        return 2.0 * FIELD_MIDLINE_X - x;
    }

    private static double mirrorHeadingDeg(double headingDeg) {
        return normalizeDeg(180.0 - headingDeg);
    }

    private static double normalizeDeg(double deg) {
        deg %= 360.0;

        if (deg >= 180.0) {
            deg -= 360.0;
        }

        if (deg < -180.0) {
            deg += 360.0;
        }

        return deg;
    }
}