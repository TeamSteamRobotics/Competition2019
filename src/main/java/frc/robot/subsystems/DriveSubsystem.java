package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

public class DriveSubsystem extends Subsystem {
    public WPI_TalonSRX leftBottom = new WPI_TalonSRX(RobotMap.leftBottom);
    public WPI_TalonSRX leftTop = new WPI_TalonSRX(RobotMap.leftTop);
    public SpeedControllerGroup left = new SpeedControllerGroup(leftBottom, leftTop);

    public WPI_TalonSRX rightBottom = new WPI_TalonSRX(RobotMap.rightBottom);
    public WPI_TalonSRX rightTop = new WPI_TalonSRX(RobotMap.rightTop);
    public SpeedControllerGroup right = new SpeedControllerGroup(rightTop, rightBottom);

    DifferentialDrive drive = new DifferentialDrive(left, right);


    public void drive (double xSpeed, double zRotation){
       drive.arcadeDrive(xSpeed, zRotation);

    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }
}

