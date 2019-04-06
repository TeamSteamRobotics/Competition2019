package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

public class DirectArmControl extends Command {
    public double speed = 4; //max speed in inches per second in the r and y direction.
    public double maxWristSpeed = 20;//maximum speed of wrist in deg/s.
    public double maxThetaSpeed = .25;//angular speed of arm in rad/s

    public double deadZone = .25;

    public double dt = .02;

    public double r;
    public double y;
    public double theta;// in radians
    public double groundAngle; //this one, this ONE value, is in degrees, not radians. It's the only one.
    public boolean flipped = false;

    public DirectArmControl() { requires(Robot.armSubsystem); }

    public void initialize() {
        //starting coordinates.
        r = 0;
        y = 6;
        theta = 0;
        groundAngle = 0;
        flipped = false;
        Robot.armSubsystem.wrist.setSelectedSensorPosition(0);
    }

    public void execute() {
        double rSpeed = -Robot.m_oi.armController.getY(Hand.kRight);
        if (Math.abs(rSpeed) < deadZone) { rSpeed = 0; }
        double ySpeed = -Robot.m_oi.armController.getY(Hand.kLeft);
        if (Math.abs(ySpeed) < deadZone) { ySpeed = 0; }
        double thetaSpeed = (Robot.m_oi.armController.getX(Hand.kLeft));
        if ((Math.abs(thetaSpeed)) < deadZone) { thetaSpeed = 0; }

        double wristSpeed = ((double)(Robot.m_oi.armController.getPOV(0) - 90.0) / 90.0);
        if(Math.abs(wristSpeed) != 1){
            wristSpeed = 0;
        }

        rSpeed *= speed;
        ySpeed *= speed;
        thetaSpeed *= speed;

        wristSpeed *= maxWristSpeed;

        r += rSpeed * dt;
        y += ySpeed * dt;
        theta += thetaSpeed * dt;
        groundAngle += wristSpeed * dt;

        //Robot.armSubsystem.wrist.set(ControlMode.Position, 0);

        //System.out.println("r: "+r+"\ny: "+y+"\ntheta: "+theta);
        //Robot.armSubsystem.setWristAngle(theta);
        Robot.armSubsystem.setWristCoordinates(theta, r, y, groundAngle, true);
        //DriverStation.reportWarning(""+Math.toDegrees(Robot.armSubsystem.elbow.getClosedLoopTarget()), false);
        //Robot.armSubsystem.printAnglesForCoordinates(theta, r, y, 0, false);
    }

    public boolean isFinished() { return false; }

    public void end() {}

    public void interrupted() {}
}