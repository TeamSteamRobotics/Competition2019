package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class LazySPX extends VictorSPX {

    private ControlMode lastControlMode = null;
    private double lastValue = Double.NaN;

    public LazySPX(int deviceNumber) { super(deviceNumber); }

    public void set(ControlMode controlMode, double value) {
        if (value != lastValue || controlMode != lastControlMode) { super.set(controlMode, value); }
    }
}