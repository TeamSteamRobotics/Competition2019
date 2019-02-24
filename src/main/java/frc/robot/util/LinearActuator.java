package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class LinearActuator extends WPI_TalonSRX {
    public static final double minActuatorLength = 17.43;
    public static final double maxActuatorLength = 29.43;

    public double offsetConstant;

    public LinearActuator(int id, double offsetConstant) {
        super(id);

        this.offsetConstant = offsetConstant;

        configSelectedFeedbackSensor(FeedbackDevice.Analog);

        configContinuousCurrentLimit(2); // TODO increase
        configPeakCurrentLimit(2);       // TODO increase
        enableCurrentLimit(true);

        selectProfileSlot(0, 0);
        config_kP(0, 30);
        config_kD(0, 1.5);
        config_kI(0, 0);
        config_kF(0, 0);
        //configMotionAcceleration(7);//~1 in/s/s
        //configMotionCruiseVelocity(22);//~3 in/s
    }
    public void setInches(double inches) {
        inches = Utils.clamp(minActuatorLength, maxActuatorLength, inches);
        set(ControlMode.Position, (inches - offsetConstant) / .0132);
    }

    public double getLength() { return getSelectedSensorPosition() * .0132 + offsetConstant; }

    public double getError() { return getClosedLoopError() * .0132; }

    public void setSensorPosition(double inches) {
        setSelectedSensorPosition((int)((inches - offsetConstant) / .0132));
    }
}