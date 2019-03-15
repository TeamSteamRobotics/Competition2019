/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotMap;
import frc.robot.commands.DirectArmControl;
import frc.robot.commands.ShuffleboardArmCommand;
import frc.robot.util.ArmPreset;
import frc.robot.util.Elbow;
import frc.robot.util.LinearActuator;
import frc.robot.util.Utils;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    static double handleLength = 5.64; //distance between the end of the handle on the arm and the central pivot point.
    static double actuator2PivotDistance = 25.172; //distance between the base of actuator 1 and the central pivot point.
    static double thetaOffset = 1.192; //angle between the actuator 1 base, central pivot point, and the end of the handle when theta1 is zero. About 68 degrees.

    static double[] robotAttachmentPoint = {-21, -2.125, 10.25}; //x, y, and z coordinates of the base of actuator 2 in robot space.
    static double pivotLength = 1.5; //distance between the actuators' y axis and z axis rotation.
    static double[] armAttachmentPoint = {11, 1.75, 0}; //x, y, z coordinates in arm space of where actuator 2 is attached to the arm.
    static double l1 = 35.75;                     //length of bottom section of arm
    static double l2 = 35.75;                     //length of top section of arm

    public LinearActuator leftActuator = new LinearActuator(RobotMap.leftActuator, 17.62);
    public LinearActuator rightActuator = new LinearActuator(RobotMap.rightActuator, 17.406);

    public Elbow elbow = new Elbow(RobotMap.elbow, 279);
    TalonSRX wrist = new TalonSRX(RobotMap.wrist);

    public double flipRadiusChange = 18.5;

    public HashMap<String, ArmPreset> presets = new HashMap<String, ArmPreset>();

    NetworkTableEntry leftLengthNTEntry;
    NetworkTableEntry rightLengthNTEntry;
    NetworkTableEntry leftErrorNTEntry;
    NetworkTableEntry rightErrorNTEntry;
    NetworkTableEntry elbowAngleEntry;
    NetworkTableEntry elbowErrorEntry;

    public ArmSubsystem() {
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

        configureWrist();

        leftLengthNTEntry = Shuffleboard.getTab("Arm Control")
                                .add("Left Length", 23)
                                .withWidget(BuiltInWidgets.kNumberBar)
                                .withProperties(Map.of("min", 17.43, "max", 29.43))
                                .getEntry();
        rightLengthNTEntry = Shuffleboard.getTab("Arm Control")
                                 .add("Right Length", 23)
                                 .withWidget(BuiltInWidgets.kNumberBar)
                                 .withProperties(Map.of("min", 17.43, "max", 29.43))
                                 .getEntry();
        leftErrorNTEntry = Shuffleboard.getTab("Arm Control")
                               .add("Left Error", -0.1)
                               .withWidget(BuiltInWidgets.kGraph)
                               .withProperties(Map.of("min", -10, "max", 10))
                               .getEntry();
        rightErrorNTEntry = Shuffleboard.getTab("Arm Control")
                                .add("Right Error", -0.1)
                                .withWidget(BuiltInWidgets.kGraph)
                                .withProperties(Map.of("min", -10, "max", 10))
                                .getEntry();
        elbowAngleEntry = Shuffleboard.getTab("Arm Control")
                                .add("Elbow Angle", -2.6866).getEntry();
        elbowErrorEntry = Shuffleboard.getTab("Arm Control")
                                .add("Elbow Error", 0).getEntry();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DirectArmControl());
    }

    public void configureWrist() {
        wrist.selectProfileSlot(0, 0);

        wrist.config_kP(0, .125);
        wrist.config_kD(0, 0);
        wrist.config_kI(0, 0);
    }

    public void setShoulderLengths(double leftLength, double rightLength) {
        leftLengthNTEntry.setNumber(leftLength);
        rightLengthNTEntry.setNumber(rightLength);

        leftErrorNTEntry.setNumber(leftActuator.getError());
        rightErrorNTEntry.setNumber(rightActuator.getError());

        leftActuator.setInches(leftLength);
        rightActuator.setInches(rightLength);
    }

    public void setShoulderAngles(double theta1, double theta2) {
        //theta1 is the direction the arm is facing relative to the robot.
        theta1 = Utils.clamp(-Math.PI / 6, Math.PI / 6, theta1);
        //theta2 is the arm's upwards angle from the ground
        theta2 = Utils.clamp(Math.PI / 4, 3 * Math.PI / 4, theta2);

        double leftLength = Math.sqrt(
            handleLength * handleLength + actuator2PivotDistance * actuator2PivotDistance - 2 * handleLength * actuator2PivotDistance * Math.cos(thetaOffset + theta1)) - pivotLength;

        //right arm attachment's position in robot space
        double rightX =
            armAttachmentPoint[0] * Math.cos(theta1) * Math.cos(theta2) + armAttachmentPoint[1] * -Math.cos(theta1) * Math.sin(theta2);
        double rightY = 
            armAttachmentPoint[0] * Math.sin(theta2) + armAttachmentPoint[1] * Math.cos(theta2);
        double rightZ =
            armAttachmentPoint[0] * Math.sin(theta1) * Math.cos(theta2) + armAttachmentPoint[1] * -Math.sin(theta1) * Math.sin(theta2);

        //finding the distance between the attachment point and the base of the right linear actuator.
        double rightLength = Math.hypot(rightY - robotAttachmentPoint[1], Math.hypot(rightX - robotAttachmentPoint[0], rightZ - robotAttachmentPoint[2]) - pivotLength);

        setShoulderLengths(leftLength, rightLength);
    }

    public void setElbowAngle(double theta) {
        theta = Utils.clamp(-2.6, 2.6, theta);

        elbow.setAngle(theta);
        elbowAngleEntry.setDouble(theta);
        elbowErrorEntry.setDouble(elbow.getError());
    }

    @Deprecated
    public void manualElbow(double power) {
        elbow.set(ControlMode.PercentOutput, power);
    }

    public void setWristAngle(double theta) {
        theta = Utils.clamp(-3 * Math.PI / 4, 3 * Math.PI / 4, theta);

        wrist.set(ControlMode.Position,
                  theta * 4096 /*counts per rotation*/ * 1 / (Math.PI * 2) /*rotations per radian*/);
    }

    public void setWristCoordinates(double theta, double r, double y, double manipulatorAngle, boolean isFlipped) {
        //r is the end of the arm's horizontal distance from its base.
        r = Utils.clamp(0, 30, r);
        //theta is the direction the arm should face.
        theta = Utils.clamp(-Math.PI / 6, Math.PI / 6, theta);
        //y is the height of the end of the arm above the ground.
        y = Utils.clamp(0, 70, y);

        //if the robot is currently flipped, reverse the direction of the arm and extend it to put the manipulator in (approximately) the same place relative to the edge of the robot.
        //also flip over manipulator angle to face the back of the robot.
        if (isFlipped) {
            r = -r - flipRadiusChange;
            manipulatorAngle = 180 - manipulatorAngle;
        }

        //make sure the position isn't too far away to reach
        if (Math.hypot(r, y) > l1 + l2) {
            r *= (l1 + l2) / Math.hypot(r, y);
            y *= (l1 + l2) / Math.hypot(r, y);
        }

        double elbowAngle = -Math.acos((r * r + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        //ensure the robot will reach up and out to the target position, not out and up.
        if (isFlipped) { elbowAngle *= -1; }
        double theta2 = Math.atan2(y, r) - Math.atan2(l2 * Math.sin(elbowAngle), l1 + l2 * Math.cos(elbowAngle));

        setElbowAngle(elbowAngle);
        setShoulderAngles(theta, theta2);
        //setWristAngle(Math.toRadians(manipulatorAngle) - elbowAngle -
        //              theta2); //turn manipulator to the correct direction relative to the ground.
    }

    public void printAnglesForCoordinates(double theta, double r, double y, double manipulatorAngle, boolean isFlipped){
        r = Utils.clamp(0, 30, r);
        //theta is the direction the arm should face.
        theta = Utils.clamp(-Math.PI / 6, Math.PI / 6, theta);
        //y is the height of the end of the arm above the ground.
        y = Utils.clamp(0, 70, y);

        //if the robot is currently flipped, reverse the direction of the arm and extend it to put the manipulator in (approximately) the same place relative to the edge of the robot.
        //also flip over manipulator angle to face the back of the robot.
        if (isFlipped) {
            r = -r - flipRadiusChange;
            manipulatorAngle = 180 - manipulatorAngle;
        }

        //make sure the position isn't too far away to reach
        if (Math.hypot(r, y) > l1 + l2) {
            r *= (l1 + l2) / Math.hypot(r, y);
            y *= (l1 + l2) / Math.hypot(r, y);
        }

        double elbowAngle = -Math.acos((r * r + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        //ensure the robot will reach up and out to the target position, not out and up.
        if (isFlipped) { elbowAngle *= -1; }
        double theta2 = Math.atan2(y, r) - Math.atan2(l2 * Math.sin(elbowAngle), l1 + l2 * Math.cos(elbowAngle));

        System.out.println("elbowAngle: "+Math.toDegrees(elbowAngle)+"\ntheta2: "+Math.toDegrees(theta2)+"\ntheta1: "+Math.toDegrees(theta));
    }
}
