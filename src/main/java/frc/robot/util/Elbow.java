package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elbow extends TalonSRX {
    private double unitsPerRadian = 160.0536;
    private double startingAngle = 2.67;

    public Elbow(int id) {
        super(id);

        setSensorPhase(true);
        
        selectProfileSlot(0, 0);

        setSelectedSensorPosition((int)(-startingAngle * unitsPerRadian));

        config_kP(0, 5);
        config_kD(0, 50);

        configContinuousCurrentLimit(5);
        enableCurrentLimit(true);
    }

    public void setAngle(double radians) {
            set(ControlMode.Position, radians * unitsPerRadian);
    }

    public double getAngle() { return (getSelectedSensorPosition()) / unitsPerRadian; }

    public double getError() { return (getClosedLoopError()) / unitsPerRadian; }
}