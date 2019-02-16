package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

public class MonitorSuctionState extends Command {

    RobotState robotState;

    public MonitorSuctionState() {
        requires(Robot.suctionSubsystem);
    }

    public void initialize() {
        robotState = RobotState.getInstance();
    }

    public void execute() {
        switch (robotState.getSuctionState()) {
            case Release:
                Robot.suctionSubsystem.divertRelease();
                Robot.suctionSubsystem.setSuctionPower(0);
                break;
            case Hold:
                switch (robotState.getMode()){
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
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
