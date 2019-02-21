/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.Mode;
import frc.robot.util.ArmPreset;

public class MonitorArmState extends Command {

  RobotState robotState;

  public MonitorArmState() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    robotState = RobotState.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    String presetName = "";
    switch(robotState.getMode()){
      case Hatch:
        presetName += "hatch";
        break;
      case Cargo:
        presetName += "cargo";
        break;
    }
    switch(robotState.getPreset()){
      case Floor:
        presetName += "Floor";
        break;
      case Ship:
        presetName += "Ship";
        break;
      case Loading:
        presetName += "Load";
        break;
      case Rocket1:
        presetName += "Rock1";
        break;
      case Rocket2:
        presetName += "Rock2";
        break;
      case Rocket3:
        presetName += "Rock3";
        break;
    }

    ArmPreset preset = Robot.armSubsystem.presets.get(presetName);

    double[] offsets = new double[3];
    offsets[0]=Robot.m_oi.armController.getX(Hand.kLeft);
    if(preset.manipulatorAngle == -90){
      offsets[1]=Robot.m_oi.armController.getY(Hand.kRight);
      offsets[2]=Robot.m_oi.armController.getY(Hand.kLeft);
    }else{
      offsets[2]=Robot.m_oi.armController.getY(Hand.kRight);
      offsets[1]=Robot.m_oi.armController.getY(Hand.kLeft);
    }
    
    double manipulatorAngle = preset.manipulatorAngle;
    if(robotState.getMode() == Mode.Cargo){
        manipulatorAngle -= 35;
    }

    Robot.armSubsystem.setWristCoordinates(offsets[0], preset.r + offsets[2], preset.y + offsets[1], manipulatorAngle, robotState.isFlipped());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
