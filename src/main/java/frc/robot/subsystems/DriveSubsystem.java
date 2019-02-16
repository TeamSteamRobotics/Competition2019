package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.util.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
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

    private boolean isRamping = false;

    public void initDefaultCommand() { setDefaultCommand(new Drive()); }

    public DriveSubsystem() {
        left = new Side("left");
        right = new Side("right");
        left.reset();
        right.reset();
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        odo = new Odometry(this);
        //odo.setPose(14, 0, 0);
        telemetry = new Telemetry();
        diffDrive = new DifferentialDrive(left.master, right.master);
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

    public class Side {

        final WPI_TalonSRX master;
        final WPI_VictorSPX follower;

        Side(String side) {
            if (side.equals("left")) {
                master = new WPI_TalonSRX(RobotMap.leftDrive);
                follower = new WPI_VictorSPX(RobotMap.leftFollower);
            } else {
                master = new WPI_TalonSRX(RobotMap.rightDrive);
                follower = new WPI_VictorSPX(RobotMap.rightFollower);
                //master.setInverted(true);
                //follower.setInverted(true);
            }
            reset();

            follower.follow(master);
        }

        public void set(double percentOutput) {
            percentOutput = Math.max(-1, Math.min(1, percentOutput));
            master.set(ControlMode.PercentOutput, percentOutput);
            //follower.set(ControlMode.PercentOutput, percentOutput);
        }

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