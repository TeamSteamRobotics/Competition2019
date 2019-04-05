package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

public class DirectArmControl extends Command {
    public double speed = 4; //max speed in inches per second in each direction.

    public double dt = .02;

    //starting coordinates. Close to when bottom section is vertical and top section is full down and forwards.
    public double r = 19.5;
    public double y = 22.75;
    public double theta = 0;
    public boolean flipped = false;

    public DirectArmControl() { requires(Robot.armSubsystem); }

    public void initialize() {
        r = 19.5;
        y = 22.75;
        theta = 0;
        flipped = false;
    }

    public void execute() {
        double rSpeed = -Robot.m_oi.armController.getY(Hand.kRight) * speed;
        if (Math.abs(rSpeed) < .5) { rSpeed = 0; }
        double ySpeed = -Robot.m_oi.armController.getY(Hand.kLeft) * speed;
        if (Math.abs(ySpeed) < .5) { ySpeed = 0; }
        double thetaSpeed = (Robot.m_oi.armController.getX(Hand.kLeft) / 16) *
                            speed; //adjust angular speed based on radius to maintain max speed.
        if ((Math.abs(thetaSpeed)) < .0625) { thetaSpeed = 0; }


        r += rSpeed * dt;
        y += ySpeed * dt;
        theta += thetaSpeed * dt;

        //System.out.println("r: "+r+"\ny: "+y+"\ntheta: "+theta);
        //Robot.armSubsystem.setWristAngle(theta);
        Robot.armSubsystem.setWristCoordinates(theta, r, y, 0, true);
        //DriverStation.reportWarning(""+Math.toDegrees(Robot.armSubsystem.elbow.getClosedLoopTarget()), false);
        //Robot.armSubsystem.printAnglesForCoordinates(theta, r, y, 0, false);
    }

    public boolean isFinished() { return false; }

    public void end() {}

    public void interrupted() {}
}