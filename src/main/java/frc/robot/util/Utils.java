package frc.robot.util;

public class Utils {
    public static double clamp(double min, double max, double value) { return Math.max(min, Math.min(max, value)); }
}