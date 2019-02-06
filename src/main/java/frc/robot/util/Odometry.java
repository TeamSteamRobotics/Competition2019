package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Pose2D;

public class Odometry {
    private DriveSubsystem subsystem;

    private volatile Pose2D pose;

    private Thread odometryThread = new Thread(() -> {
        double lastPos = (subsystem.left.getDistance() + subsystem.right.getDistance()) / 2;
        while (true) {
            double currentPos = (subsystem.left.getDistance() + subsystem.right.getDistance()) / 2;
            double dPos = currentPos - lastPos;
            synchronized (this) {
                pose.theta = -subsystem.ahrs.getYaw() * 3.1415926 / 180.0;
                pose.x += Math.cos(pose.theta) * dPos;
                pose.y += Math.sin(pose.theta) * dPos;
            }
            lastPos = currentPos;
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) { e.printStackTrace(); }
        }
    });

    public Odometry(DriveSubsystem drivetrain) {
        subsystem = drivetrain;
        synchronized (this) { pose = new Pose2D(0, 0, 0); }
        odometryThread.start();
    }

    public void setPose(Pose2D newPose) {
        synchronized (this) { pose = newPose.copy(); }
    }

    public void setPose(double x, double y, double theta) {
        synchronized (this) { pose = new Pose2D(x, y, theta); }
    }

    public Pose2D getPose() { return pose.copy(); }

    public double getX() { return pose.x; }

    public double getY() { return pose.y; }

    public double getTheta() { return pose.theta; }
}