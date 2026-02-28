package org.firstinspires.ftc.teamcode.Robot.Subsystems;

public final class SpinDexHandoff {
    private static boolean hasSaved = false;
    private static int savedTicks = 0;

    private SpinDexHandoff() {}

    public static void save(int ticks) {
        savedTicks = ticks;
        hasSaved = true;
    }

    public static boolean hasSaved() {
        return hasSaved;
    }

    public static int getSavedTicks() {
        return savedTicks;
    }

    public static void clear() {
        hasSaved = false;
        savedTicks = 0;
    }
}