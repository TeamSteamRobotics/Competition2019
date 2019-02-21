/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MonitorArmState;
import frc.robot.util.ArmPreset;
import frc.robot.util.LinearActuator;
import frc.robot.util.Utils;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static double xOffset = 21;//how far behind the arm the linear actuators are.
  static double zOffset = 9.25;//how far to the side of the arm either linear actuator is.
  static double yOffset = 2;//how far below the arm base the linear actuator base is.
  static double pivotLength = 1.5;//distance between the actuators' y axis and z axis rotation.
  static double armAttachmentSeparation = 2.032;//how far the actuators' attachment points are from the center line of the arm.
  static double armAttachmentHeight = 11;//how high along the arm the actuators are attached
  static double l1 = 36;//length of bottom section of arm
  static double l2 = 39;//length of top section of arm

  LinearActuator leftActuator = new LinearActuator(RobotMap.leftActuator);
  LinearActuator rightActuator = new LinearActuator(RobotMap.rightActuator);
  

  TalonSRX elbow = new TalonSRX(RobotMap.elbow);
  TalonSRX wrist = new TalonSRX(RobotMap.wrist);

  
  public double flipRadiusChange = 18.5;

  public HashMap<String, ArmPreset> presets = new HashMap<String, ArmPreset>();

  public ArmSubsystem(){
    presets.put("hatchFloor", new ArmPreset(12, 12, -90));
    presets.put("hatchShip", new ArmPreset(6, 18, 0));
    presets.put("hatchLoad", new ArmPreset(6, 18, 0));
    presets.put("hatchRock1", new ArmPreset(6, 18, 0));
    presets.put("hatchRock2", new ArmPreset(6, 38, 0));
    presets.put("hatchRock3", new ArmPreset(6, 58, 0));
    presets.put("cargoFloor", new ArmPreset(12, 22, -90));
    presets.put("cargoShip", new ArmPreset(12, 38, -90));
    presets.put("cargoLoad", new ArmPreset(6, 28, 0));
    presets.put("cargoRock1", new ArmPreset(6, 28, 0));
    presets.put("cargoRock2", new ArmPreset(6, 48, 0));
    presets.put("cargoRock3", new ArmPreset(6, 68, 0));
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MonitorArmState());
  }

  public void configureElbow(){
    elbow.selectProfileSlot(0, 0);

    elbow.config_kP(0, .125);
    elbow.config_kD(0, 0);
    elbow.config_kI(0, 0);
  }

  public void configureWrist(){
    wrist.selectProfileSlot(0, 0);

    wrist.config_kP(0, .125);
    wrist.config_kD(0, 0);
    wrist.config_kI(0, 0);
  }

  public void setShoulderLengths(double leftLength, double rightLength){
    leftActuator.setInches(leftLength);
    rightActuator.setInches(rightLength);
  }

  public void setShoulderAngles(double theta1, double theta2){
    //theta1 is the direction the arm is facing relative to the robot.
    theta1 = Utils.clamp(-Math.PI / 6, Math.PI / 6, theta1);
    //theta2 is the arm's upwards angle from the ground
    theta2 = Utils.clamp(Math.PI/4, 3*Math.PI/4, theta2);

    //left arm attachment's position in robot coordinates
    double leftX = armAttachmentHeight*Math.cos(theta1)*Math.cos(theta2) + armAttachmentSeparation * -Math.sin(theta1);
    double leftY = armAttachmentHeight*Math.sin(theta2);
    double leftZ = armAttachmentHeight*Math.sin(theta1)*Math.cos(theta2) + armAttachmentSeparation * Math.cos(theta1);

    //right arm attachment's position in robot coordinates
    double rightX = armAttachmentHeight*Math.cos(theta1)*Math.cos(theta2) - armAttachmentSeparation * -Math.sin(theta1);
    double rightY = armAttachmentHeight*Math.sin(theta2);
    double rightZ = armAttachmentHeight*Math.sin(theta1)*Math.cos(theta2) - armAttachmentSeparation * Math.cos(theta1);

    //finding the distance between the attachment points and the base of the corresponding linear actuator.
    double leftLength = Math.hypot(leftY + yOffset, Math.hypot(leftX + xOffset, leftZ + zOffset) - pivotLength);
    double rightLength = Math.hypot(rightY + yOffset, Math.hypot(rightX + xOffset, rightZ - zOffset) - pivotLength);

    setShoulderLengths(leftLength, rightLength);
  }

  public void setElbowAngle(double theta){
    theta = Utils.clamp(-3*Math.PI/4, 3*Math.PI/4, theta);

    elbow.set(ControlMode.Position, theta * 4096 /*counts per rotation*/ * 1 / (Math.PI * 2) /*rotations per radian*/);
  }

  public void setWristAngle(double theta){
    theta = Utils.clamp(-3*Math.PI/4, 3*Math.PI/4, theta);

    wrist.set(ControlMode.Position, theta * 4096 /*counts per rotation*/ * 1 / (Math.PI * 2) /*rotations per radian*/);
  }

  public void setWristCoordinates(double theta, double r, double y, double manipulatorAngle, boolean isFlipped){
    //r is the end of the arm's horizontal distance from its base.
    r = Utils.clamp(0, 30, r);
    //theta is the direction the arm should face.
    theta = Utils.clamp(-Math.PI / 6, Math.PI / 6, theta);
    //y is the height of the end of the arm above the ground.
    y = Utils.clamp(0, 70, y);

    //if the robot is currently flipped, reverse the direction of the arm and extend it to put the manipulator in (approximately) the same place relative to the edge of the robot.
    //also flip over manipulator angle to face the back of the robot.
    if(isFlipped){
      r = -r - flipRadiusChange;
      manipulatorAngle = 180 - manipulatorAngle;
    }

    //make sure the position isn't too far away to reach
    if(Math.hypot(r, y) > l1 + l2){
      r *= (l1+l2)/Math.hypot(r, y);
      y *= (l1+l2)/Math.hypot(r, y);
    }

    double elbowAngle = Math.acos((r*r + y*y - l1*l1 - l2*l2)/(2*l1*l2));
    //ensure the robot will reach up and out to the target position, not out and up.
    if(isFlipped){
      elbowAngle *= -1;
    }
    double theta2 = Math.atan2(y, r) + Math.atan2(l2*Math.sin(elbowAngle), l1 + l2*Math.cos(elbowAngle));
    
    setElbowAngle(elbowAngle);
    setShoulderAngles(theta, theta2);
    setWristAngle(Math.toRadians(manipulatorAngle) - elbowAngle - theta2);//turn manipulator to the correct direction relative to the ground.
  }
}
