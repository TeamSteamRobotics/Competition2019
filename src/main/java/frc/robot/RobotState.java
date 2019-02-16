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

    public enum FlipState {
        Normal, Flipped
    }

    public double[] armPositionOffsets = new double[3];//{thetaOffset, yOffset, rOffset}

    private static RobotState INSTANCE = null;

    private Mode mode;

    private SuctionState suctionState;

    private ArmPosition preset;

    private FlipState flip;

    private RobotState() {
        mode = Mode.Hatch;
        suctionState = SuctionState.Release;
        preset = ArmPosition.Loading;
        flip = FlipState.Normal;
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

    public void setFlip(FlipState flip){
        this.flip = flip;
    }
    public FlipState getFlipState(){
        return flip;
    }

    public void setOffsets(double tOffset, double yOffset, double rOffset){
        armPositionOffsets[0] = tOffset;
        armPositionOffsets[1] = yOffset;
        armPositionOffsets[2] = rOffset;
    }
    public double[] getOffsets(){
        return armPositionOffsets;
    }
}
