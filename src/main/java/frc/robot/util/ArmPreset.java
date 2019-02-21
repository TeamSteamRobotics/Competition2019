package frc.robot.util;

public class ArmPreset {
    public double r;
    public double y;
    public double manipulatorAngle; //angle of manipulator relative to ground. 0 if horizontal, 90 to point down

    public ArmPreset(double r, double y, double manipulatorAngle) {
        this.r = r;
        this.y = y;
        this.manipulatorAngle = manipulatorAngle;
    }
}