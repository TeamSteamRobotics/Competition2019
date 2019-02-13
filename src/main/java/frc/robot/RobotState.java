package frc.robot;

public class RobotState {

    public enum State {
        Idle, Hatch, Ball, Release
    }

    private static RobotState INSTANCE = null;

    private State state;

    private RobotState() {
        state = State.Idle;
    }

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void setState(State state) {
        this.state = state;
    }
    public State getState() {
        return state;
    }
}
