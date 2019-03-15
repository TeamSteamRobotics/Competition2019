/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Utils;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {

    static double handleLength = 5.64; //distance between the end of the handle on the arm and the central pivot point.
    static double actuator2PivotDistance = 25.172; //distance between the base of actuator 1 and the central pivot point.
    static double thetaOffset = 1.192; //angle between the actuator 1 base, central pivot point, and the end of the handle when theta1 is zero. About 68 degrees.
    private Main() {}

    /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
    public static void main(String... args) {
        /**/RobotBase.startRobot(Robot::new); /*/
        //right arm attachment's position in robot space
        double[] robotAttachmentPoint = {-21, -2.125, 10.25}; //x, y, and z coordinates of the base of actuator 2 in robot space.
        double pivotLength = 1.5; //distance between the actuators' y axis and z axis rotation.
        double[] armAttachmentPoint = {11, 1.75, 0};
        
        double theta1 = 0;
        double theta2 = Math.PI / 2;

        double rightX =
            armAttachmentPoint[0] * Math.cos(theta1) * Math.cos(theta2) + armAttachmentPoint[1] * -Math.cos(theta1) * Math.sin(theta2);
        double rightY = 
            armAttachmentPoint[0] * Math.sin(theta2) + armAttachmentPoint[1] * Math.cos(theta2);
        double rightZ =
            armAttachmentPoint[0] * Math.sin(theta1) * Math.cos(theta2) + armAttachmentPoint[1] * -Math.sin(theta1) * Math.sin(theta2);

        double rightLength = Math.hypot(rightY - robotAttachmentPoint[1], Math.hypot(rightX - robotAttachmentPoint[0], rightZ - robotAttachmentPoint[2]) - pivotLength);
        System.out.println(rightLength);

        double leftLength = Math.sqrt(
            handleLength * handleLength + actuator2PivotDistance * actuator2PivotDistance - 2 * handleLength * actuator2PivotDistance * Math.cos(thetaOffset + theta1));

        System.out.println(leftLength);/**/
    }
}
