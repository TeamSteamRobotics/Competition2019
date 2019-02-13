package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LinearActuator extends TalonSRX {
    public static final double minActuatorLength = 17.43;
    public static final double maxActuatorLength = 26.43;

    public LinearActuator(int port){
        super(port);
        configSelectedFeedbackSensor(FeedbackDevice.Analog);

        configContinuousCurrentLimit(5);
        configPeakCurrentLimit(5);
        enableCurrentLimit(true);

        selectProfileSlot(0, 0);
        config_kP(0, 30);
        config_kD(0, 1.5);
        config_kI(0, 0);
        config_kF(0, 0);
        //configMotionAcceleration(7);//~1 in/s/s
        //configMotionCruiseVelocity(22);//~3 in/s
    }
    public void setInches(double inches){
        inches = Utils.clamp(minActuatorLength, maxActuatorLength, inches);
        set(ControlMode.Position, inches/.0135);
    }

    public double getLength(){
        return getSelectedSensorPosition() * .0135;
    }

    public void setSensorPosition(double inches){
        setSelectedSensorPosition((int)(inches / .0135));
    }

}