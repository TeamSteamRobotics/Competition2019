package frc.robot;

public class RobotState {

    public enum Mode {
        Hatch, Cargo
    }

    public enum SuctionState {
        Hold, Release
    }

    public enum ArmPosition {
        Loading, Floor, Ship, Rocket1, Rocket2, Rocket3
    }

    private boolean isFlipped;

    //public double[] armPositionOffsets = new double[3];//{thetaOffset, yOffset, rOffset}

    private static RobotState INSTANCE = null;

    private Mode mode;

    private SuctionState suctionState;

    private ArmPosition preset;

    private RobotState() {
        mode = Mode.Hatch;
        suctionState = SuctionState.Release;
        preset = ArmPosition.Loading;
        isFlipped = false;
    }

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void setMode(Mode mode){
        this.mode = mode;
    }
    public Mode getMode(){
        return mode;
    }

    public void setSuctionState(SuctionState state) {
        this.suctionState = state;
    }
    public SuctionState getSuctionState() {
        return suctionState;
    }

    public void setPreset(ArmPosition preset){
        this.preset = preset;
    }
    public ArmPosition getPreset(){
        return preset;
    }

    public void setFlip(boolean flip){
        isFlipped = flip;
    }
    public boolean isFlipped(){
        return isFlipped;
    }

    /*public void setOffsets(double tOffset, double yOffset, double rOffset){
        armPositionOffsets[0] = tOffset;
        armPositionOffsets[1] = yOffset;
        armPositionOffsets[2] = rOffset;
    }
    public double[] getOffsets(){
        return armPositionOffsets;
    }*/
}
