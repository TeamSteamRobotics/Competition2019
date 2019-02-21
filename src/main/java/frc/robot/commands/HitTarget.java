/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HitTarget extends Command {
    public double xError;
    public double lastRelativeYaw;
    public double lastVisionGyro;
    public double lastLeftEncoderDistance;
    public double lastRightEncoderDistance;
    public boolean hasSeenTarget = false;

    public HitTarget() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        requires(Robot.visionSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        hasSeenTarget = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        double[] tvec = Robot.visionSubsystem.getTVec();
        if (tvec.length > 0 && tvec[2] != 0.0) {
            lastRelativeYaw = Robot.visionSubsystem.getYaw();
            lastVisionGyro = Robot.driveSubsystem.ahrs.getAngle();
            xError = tvec[0] + 12.5;
            double zError = tvec[2];
            double xCorrectionAngle = Math.atan2(xError, zError) * 180.0 / Math.PI;
            double targetYaw = xError * 1;
            //%DriverStation.reportWarning("targetYaw: "+targetYaw, false);
            double steerCorrection = (targetYaw - lastRelativeYaw) * -1.0 / 90.0;
            Robot.driveSubsystem.drive(-0.3, steerCorrection);
            hasSeenTarget = true;
        } else if (hasSeenTarget) { //odometry-ish stuff, but not really. Definitely should change/rework/delete this.
            double gyroChange =
                lastVisionGyro -
                Robot.driveSubsystem.ahrs.getAngle(); //how much we have turned since the last time we saw the target.
            double relativeYaw =
                lastRelativeYaw +
                gyroChange; //combining gyroChange with the last target orientation to get the (supposed) current target orientation
            double dLeft = Robot.driveSubsystem.left.getDistance() - lastLeftEncoderDistance;
            double dRight = Robot.driveSubsystem.right.getDistance() - lastRightEncoderDistance;
            double displacement = (dLeft + dRight) / 2.0;                     //odometry stuff
            xError += displacement * Math.sin(relativeYaw * Math.PI / 180.0); //how much x error changed
            double targetYaw = xError * 1.3;
            double steerCorrection = (targetYaw - relativeYaw) * -1.0 / 90.0;
            Robot.driveSubsystem.drive(-0.2, steerCorrection);
            DriverStation.reportError("xError: " + xError, false);
        }
        lastLeftEncoderDistance = Robot.driveSubsystem.left.getDistance();
        lastRightEncoderDistance = Robot.driveSubsystem.right.getDistance();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {}
}
