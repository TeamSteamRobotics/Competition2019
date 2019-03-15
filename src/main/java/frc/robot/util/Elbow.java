package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elbow extends TalonSRX {
    private final double zeroUnits; //encoder value in native units when angle is 0.
    private double unitsPerRadian = 160.0536;

    public Elbow(int id, double zeroAngleUnits){
        super(id);

        zeroUnits = zeroAngleUnits;

        setSensorPhase(true);

        selectProfileSlot(0, 0);

        config_kP(0, 20);
        config_kD(0, 2000);

        //config_IntegralZone(0, 42);//about 15 degrees
        //configMaxIntegralAccumulator(0, 100);
        config_kI(0, 0);
    }

    public void setAngle(double radians){
        set(ControlMode.Position, (radians * unitsPerRadian) + zeroUnits);
    }

    public double getAngle(){
        return (getSelectedSensorPosition() - zeroUnits) / unitsPerRadian;
    }

    public double getError(){
        return (getClosedLoopError()) / unitsPerRadian;
    }
}