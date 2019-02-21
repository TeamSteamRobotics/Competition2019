/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Add your docs here.
 */
public class TriggerButton extends Trigger {
    XboxController controller;
    Hand hand;

    public TriggerButton(XboxController controller, Hand hand) {
        this.controller = controller;
        this.hand = hand;
    }

    @Override
    public boolean get() {
        return controller.getTriggerAxis(hand) > .5;
    }

    public void whenPressed(Command command) { whenActive(command); }

    public void whileHeld(Command command) { whileActive(command); }
}
