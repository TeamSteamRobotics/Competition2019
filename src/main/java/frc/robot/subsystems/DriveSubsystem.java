package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.util.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    public Side left;
    public Side right;
    public double wheelCircumf = 6.0 * 3.141592653;//units = inches
    public double encoderResolution = 4096.0;//4096 units per rotation
    public Odometry odo;
    public AHRS ahrs;
    public Telemetry telemetry;
    public DifferentialDrive diffDrive;
    //public Servo servo = new Servo(0); .21, .4, .71

    private boolean isRamping = false;

    public void initDefaultCommand() { setDefaultCommand(new Drive()); }

    public DriveSubsystem() {
        left = new Side(RobotMap.left);
        right = new Side(RobotMap.right);
        left.reset();
        right.reset();
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        odo = new Odometry(this);
        //odo.setPose(14, 0, 0);
        telemetry = new Telemetry();
        //servo.set
        diffDrive = new DifferentialDrive(left, right);
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

    public boolean isRamping() { return isRamping; }

    public void drive(double xSpeed, double zRotation) {
        //servo.setPosition((xSpeed + 1)/2.0);
        //DriverStation.reportError("angle: "+servo.getPosition(), false);
        diffDrive.arcadeDrive(xSpeed, zRotation);
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

    public class Side extends WPI_TalonSRX{

        final WPI_VictorSPX follower;

        Side(int id) {
            super(id);
            follower = new WPI_VictorSPX(id);
            //reset();

            follower.follow(this);
        }

        public void set(double percentOutput) {
            percentOutput = Math.max(-1, Math.min(1, percentOutput));
            set(ControlMode.PercentOutput, percentOutput);
            //follower.set(ControlMode.PercentOutput, percentOutput);
        }

        public void reset() { setSelectedSensorPosition(0); }

        void startRamping() { configOpenloopRamp(0.5, 1000); }

        void stopRamping() { configOpenloopRamp(0, 1000); }

        public int getCount() { return getSelectedSensorPosition(); }

        public double getDistance() { return ((double)getCount() / encoderResolution) * wheelCircumf; }

        public double getRate() { return getSelectedSensorVelocity(); }
    }

    public class Telemetry {
        public double leftPct, leftAmps, leftEnc, rightPct, rightAmps, rightEnc;

        public void update() {
            leftPct = left.getMotorOutputPercent();
            //leftAmps = left.master.getOutputCurrent() + left.follower.getOutputCurrent();
            leftEnc = left.get();

            rightPct = right.getMotorOutputPercent();
            //rightAmps = right.master.getOutputCurrent() + right.follower.getOutputCurrent();
            rightEnc = right.get();
        }
    }
}