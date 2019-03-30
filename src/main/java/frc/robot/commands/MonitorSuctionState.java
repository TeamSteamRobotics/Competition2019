package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

public class MonitorSuctionState extends Command {

    RobotState robotState;

    public MonitorSuctionState() { requires(Robot.suctionSubsystem); }

    public void initialize() { robotState = RobotState.getInstance(); }

    public void execute() {
        switch (robotState.getSuctionState()) {
            case Release:
                Robot.suctionSubsystem.divertRelease();
                Robot.suctionSubsystem.setSuctionPower(0);
                Robot.m_oi.armController.setRumble(RumbleType.kRightRumble, 0);
                break;
            case Hold:
            Robot.m_oi.armController.setRumble(RumbleType.kRightRumble, 1);
                switch (robotState.getMode()) {
                    case Hatch:
                        Robot.suctionSubsystem.divertHatch();
                        break;
                    case Cargo:
                        Robot.suctionSubsystem.divertBall();
                        break;
                }
                Robot.suctionSubsystem.setSuctionPower(1);
                break;
        }
        //double angle = Robot.m_oi.armController.getY(Hand.kRight) * 90 + 90;
        //Robot.suctionSubsystem.setDiverterAngle(angle);
        //DriverStation.reportWarning("angle: "+angle, false);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
