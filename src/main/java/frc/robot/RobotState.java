package frc.robot;

public class RobotState {

    public enum SuctionState {
        Idle, Hatch, Ball
    }

    private static RobotState INSTANCE = null;

    private SuctionState state;

    private RobotState() {
        state = SuctionState.Idle;
    }

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void setState(SuctionState state) {
        this.state = state;
    }
    public SuctionState getState() {
        return state;
    }
}
