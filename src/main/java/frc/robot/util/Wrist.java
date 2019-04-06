package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Wrist extends TalonSRX {
    private double unitsPerRadian = -1620.197;
    private double startingAngle = 0;

    public Wrist(int id){
        super(id);

        setSensorPhase(false);

        selectProfileSlot(0, 0);

        setSelectedSensorPosition((int)(startingAngle * unitsPerRadian));

        config_kP(0, .75);
        config_kD(0, .125);

        //config_IntegralZone(0, 42);//about 15 degrees
        //configMaxIntegralAccumulator(0, 100);
        config_kI(0, 0);
    }

    public void setAngle(double radians){
        set(ControlMode.Position, radians * unitsPerRadian);
    }

    public double getAngle(){
        return getSelectedSensorPosition() / unitsPerRadian;
    }

    public double getError(){
        return (getClosedLoopError()) / unitsPerRadian;
    }
}