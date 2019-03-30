/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.Flip;
import frc.robot.commands.HitTarget;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetRobotMode;
import frc.robot.commands.ToggleSuctionState;
import frc.robot.util.TriggerButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    public Joystick stick = new Joystick(0);

    JoystickButton goToTarget = new JoystickButton(stick, 1);

    public XboxController armController = new XboxController(1);

    POVButton floorButton = new POVButton(armController, 270);
    POVButton loadButton = new POVButton(armController, 90);
    JoystickButton shipButton = new JoystickButton(armController, 1);
    JoystickButton rocket1Button = new JoystickButton(armController, 3);
    JoystickButton rocket2Button = new JoystickButton(armController, 4);
    JoystickButton rocket3Button = new JoystickButton(armController, 2);
    JoystickButton cargoMode = new JoystickButton(armController, 5);
    JoystickButton hatchMode = new JoystickButton(armController, 6);
    TriggerButton toggleSuction = new TriggerButton(armController, Hand.kRight);
    TriggerButton flip = new TriggerButton(armController, Hand.kLeft);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
    public OI() {
        goToTarget.whileHeld(new HitTarget());

        floorButton.whenPressed(new SetArmPosition(RobotState.ArmPosition.Floor));
        loadButton.whenPressed(new SetArmPosition(RobotState.ArmPosition.Loading));
        shipButton.whenPressed(new SetArmPosition(RobotState.ArmPosition.Ship));
        rocket1Button.whenPressed(new SetArmPosition(RobotState.ArmPosition.Rocket1));
        rocket2Button.whenPressed(new SetArmPosition(RobotState.ArmPosition.Rocket2));
        rocket3Button.whenPressed(new SetArmPosition(RobotState.ArmPosition.Rocket3));
        cargoMode.whenPressed(new SetRobotMode(RobotState.Mode.Cargo));
        hatchMode.whenPressed(new SetRobotMode(RobotState.Mode.Hatch));
        toggleSuction.whenPressed(new ToggleSuctionState());
        flip.whenPressed(new Flip());
    }
}
