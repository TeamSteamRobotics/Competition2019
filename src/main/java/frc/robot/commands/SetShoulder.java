/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetShoulder extends Command {
  double theta1;
  double theta2;
  public SetShoulder(double theta1, double theta2) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.armSubsystem);
    this.theta1 = theta1;
    this.theta2 = theta2;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.armSubsystem.setShoulderAngles(theta1, theta2);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armSubsystem.setShoulderAngles(theta1, theta2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.armSubsystem.leftActuator.getError()) < .5) && (Math.abs(Robot.armSubsystem.rightActuator.getError()) < .5);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
