package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class PoseHandoff {
    private static Pose savedPose = null;
    private static boolean hasPose = false;

    public static void save(Pose pose) {
        if (pose == null) return;
        savedPose = new Pose(pose.getX(), pose.getY(), pose.getHeading()); // copy
        hasPose = true;
    }

    public static boolean hasPose() {
        return hasPose && savedPose != null;
    }

    public static Pose get() {
        return savedPose;
    }

    public static void clear() {
        savedPose = null;
        hasPose = false;
    }
}