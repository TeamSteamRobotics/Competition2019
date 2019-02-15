package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.util.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.Constants;
import frc.robot.RobotMap;
//import frc.robot.util.Odometry;
import frc.robot.commands.Drive;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    public Side left;
    public Side right;
    //WPI_TalonSRX test = new WPI_TalonSRX(0);
    public double wheelCircumf = 6.0 * 3.141592653;//units = inches
    public double encoderResolution = 4096.0;//4096 units per rotation
    public Odometry odo;
    public AHRS ahrs;
    public Telemetry telemetry;

    private boolean isRamping = false;

    public void initDefaultCommand() { setDefaultCommand(new Drive()); }

    public DriveSubsystem() {
        //test.selectProfileSlot(0, 0);
        //test.config_kP(0, .125);
        //test.config_kD(0, 0.0);
        left = new Side("left");
        right = new Side("right");
        left.reset();
        right.reset();
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        odo = new Odometry(this);
        //odo.setPose(14, 0, 0);
        telemetry = new Telemetry();
    }

    public void startRamping() {
        if (!isRamping) {
            left.startRamping();
            right.startRamping();
            isRamping = true;
        }
    }

    public void stopRamping() {
        if (isRamping) {
            left.stopRamping();
            right.stopRamping();
            isRamping = false;
        }
    }

    public void startPID() {
        left.startPID();
        right.startPID();
    }

    public void stopPID() {
        left.stopPID();
        right.stopPID();
    }

    public boolean isRamping() { return isRamping; }

    public void drive(double xSpeed, double zRotation) {
        left.follower.setInches(22 + (4 * xSpeed));
        //DriverStation.reportError("position: " + left.follower.getSelectedSensorPosition(), false);
        DriverStation.reportError("length: "+(left.follower.getLength()), false);
        /*left.master.set(ControlMode.Position, (xSpeed * 8192));
        right.master.set(ControlMode.Position, (xSpeed * 8192));
        DriverStation.reportError("error: "+left.master.getClosedLoopError(), false);*/
        /*double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        left.set(leftMotorOutput);
        right.set(rightMotorOutput);*/
    }

    public class Side {

        final TalonSRX master;
        final LinearActuator follower;
        //private final Encoder quadrature;
        private volatile double setpoint = 0;
        private volatile double lastError = 0;

        private Notifier PIDLoop = new Notifier(() -> {
            //double error = (setpoint - getRate());
            //double output = Constants.kf * setpoint;
            //output += Constants.kp * error;
            //output += Constants.kd * ((error - lastError) / 0.02);
            //set(output);
            //lastError = error;
        });

        Side(String side) {
            if (side.equals("left")) {
                master = new TalonSRX(RobotMap.leftDrive);
                follower = new LinearActuator(RobotMap.leftFollower);
                //quadrature = new Encoder(RobotMap.leftDriveEncA, RobotMap.leftDriveEncB, false);
            } else {
                master = new TalonSRX(RobotMap.rightDrive);
                follower = new LinearActuator(RobotMap.rightFollower);
                master.setInverted(true);
                follower.setInverted(true);
                //quadrature = new Encoder(RobotMap.rightDriveEncA, RobotMap.rightDriveEncB, false);
            }
            reset();

            follower.configSelectedFeedbackSensor(FeedbackDevice.Analog);
            follower.configContinuousCurrentLimit(5);
            follower.configPeakCurrentLimit(5);
            follower.enableCurrentLimit(true);
            //follower.setSelectedSensorPosition(2130);

            follower.selectProfileSlot(0, 0);
            //follower.setSensorPhase(true);
            follower.config_kP(0, 10);
            follower.config_kD(0, .5);
            follower.config_kI(0, 0);
            
            follower.config_kF(0, 0.0);

            //follower.follow(master);

            //quadrature.setDistancePerPulse(2 * Math.PI / 2048.0); //should make getRate() return rad/s
        }

        public void set(double percentOutput) {
            percentOutput = Math.max(-1, Math.min(1, percentOutput));
            master.set(ControlMode.Position, percentOutput * 4096);
        }

        public void startPID() { PIDLoop.startPeriodic(0.01); }

        public void stopPID() { PIDLoop.stop(); }

        public void setSpeed(double setpoint) { this.setpoint = setpoint; }

        public void reset() { master.setSelectedSensorPosition(0); }

        void startRamping() { master.configOpenloopRamp(0.5, 1000); }

        void stopRamping() { master.configOpenloopRamp(0, 1000); }

        public int get() { return master.getSelectedSensorPosition(); }

        public double getDistance() { return ((double)get() / encoderResolution) * wheelCircumf; }

        public double getRate() { return master.getSelectedSensorVelocity(); }
    }

    public class Telemetry {
        public double leftPct, leftAmps, leftEnc, rightPct, rightAmps, rightEnc;

        public void update() {
            leftPct = left.master.getMotorOutputPercent();
            //leftAmps = left.master.getOutputCurrent() + left.follower.getOutputCurrent();
            leftEnc = left.get();

            rightPct = right.master.getMotorOutputPercent();
            //rightAmps = right.master.getOutputCurrent() + right.follower.getOutputCurrent();
            rightEnc = right.get();
        }
    }
}