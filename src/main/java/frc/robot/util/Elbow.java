package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Notifier;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Elbow extends TalonSRX {
    private final double zeroUnits; //encoder value in native units when angle is 0.
    private double unitsPerRadian = 160.0536;

    private Supplier<Double> mastTheta;

    private double targetAngle = 0;

    private Notifier sender = new Notifier(this ::sendIt);
    private boolean sending = false;

    public Elbow(int id, double zeroAngleUnits, Supplier<Double> mastTheta) {
        super(id);

        zeroUnits = zeroAngleUnits;
        this.mastTheta = mastTheta;

        setSensorPhase(true);

        selectProfileSlot(0, 0);

        config_kP(0, 20);
        config_kD(0, 2000);

        //config_IntegralZone(0, 42);//about 15 degrees
        //configMaxIntegralAccumulator(0, 100);
        config_kI(0, 0);
    }

    public void setAngle(double radians) {
        targetAngle = radians;
        if (!sending) { sender.startPeriodic(0.02); }
    }

    public double getAngle() { return (getSelectedSensorPosition() - zeroUnits) / unitsPerRadian; }

    public double getError() { return (getClosedLoopError()) / unitsPerRadian; }

    /**
     * https://www.youtube.com/watch?v=mzOUgwsQ_hM
     */
    private void sendIt() {
        set(ControlMode.Position, targetAngle * unitsPerRadian + zeroUnits, DemandType.ArbitraryFeedForward,
            0.1 * Math.cos(mastTheta.get() + targetAngle));
    }
}