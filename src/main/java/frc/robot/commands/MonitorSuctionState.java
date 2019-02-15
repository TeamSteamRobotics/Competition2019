package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;

public class MonitorSuctionState extends Command {

    public MonitorSuctionState() {
        requires(Robot.suctionSubsystem);
    }

    public void initialize() {
    }

    public void execute() {
        RobotState.SuctionState suctionState = RobotState.getInstance().getState();
        switch (suctionState) {
            case Idle:
                Robot.suctionSubsystem.divertRelease();
                Robot.suctionSubsystem.setSuctionPower(0);
                break;
            case Ball:
                Robot.suctionSubsystem.divertBall();
                Robot.suctionSubsystem.setSuctionPower(1);
                break;
            case Hatch:
                Robot.suctionSubsystem.divertHatch();
                Robot.suctionSubsystem.setSuctionPower(1);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
