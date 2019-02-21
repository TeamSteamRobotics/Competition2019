package frc.robot.util;

public class Pose2D {
    public double x, y, theta;

    public Pose2D() {
        x = 0;
        y = 0;
        theta = 0;
    }
    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2D copy() { return new Pose2D(x, y, theta); }

    public String toString() { return "x: " + x + " y: " + y + " theta: " + theta; }
}