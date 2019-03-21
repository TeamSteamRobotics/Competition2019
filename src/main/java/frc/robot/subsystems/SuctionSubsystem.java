/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MonitorSuctionState;

public class SuctionSubsystem extends Subsystem {

    private TalonSRX vacuum = new TalonSRX(RobotMap.vacuum);
    //private Servo diverter = new Servo(0);

    private final double kDiverterHatchAngle = 0000;
    private final double kDiverterBallAngle = 0000;
    private final double kDiverterReleaseAngle = 0000;

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new MonitorSuctionState());
    }

    public void setSuctionPower(double power) { vacuum.set(ControlMode.PercentOutput, power); }

public void setDiverterAngle(double angle) { /*diverter.setAngle(angle);*/ }

    public void divertRelease() { setDiverterAngle(kDiverterReleaseAngle); }

    public void divertHatch() { setDiverterAngle(kDiverterHatchAngle); }

    public void divertBall() { setDiverterAngle(kDiverterBallAngle); }
}
