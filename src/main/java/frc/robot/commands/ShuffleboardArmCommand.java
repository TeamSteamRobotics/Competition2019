/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;
import frc.robot.RobotState;

import java.util.Map;

public class ShuffleboardArmCommand extends Command {

    RobotState robotState;

    static NetworkTableEntry thetaOne;
    static NetworkTableEntry thetaTwo;
    static NetworkTableEntry elbowAngle;

    public ShuffleboardArmCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.armSubsystem);
        
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        robotState = RobotState.getInstance();
        if(thetaOne == null){
            thetaOne = Shuffleboard.getTab("Arm Control")
                       .add("Theta One", 0)
                       .withWidget(BuiltInWidgets.kNumberSlider)
                       .withProperties(Map.of("min", -Math.PI / 6, "max", Math.PI / 6))
                       .getEntry();
            thetaTwo = Shuffleboard.getTab("Arm Control")
                           .add("Theta Two", Math.PI / 2)
                           .withWidget(BuiltInWidgets.kNumberSlider)
                           .withProperties(Map.of("min", Math.PI / 4, "max", 3 * Math.PI / 4))
                           .getEntry();
            elbowAngle = Shuffleboard.getTab("Arm Control")
                    .add("Elbow Angle", 0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -2, "max", 2))
                    .getEntry();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        DriverStation.reportWarning("elbow error: "+Robot.armSubsystem.elbow.getClosedLoopError(), false);
        Robot.armSubsystem.setShoulderAngles(thetaOne.getDouble(0), thetaTwo.getDouble(Math.PI / 2));
        Robot.armSubsystem.setElbowAngle(elbowAngle.getDouble(-2));
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
