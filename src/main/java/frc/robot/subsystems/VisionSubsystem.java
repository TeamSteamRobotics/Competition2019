/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

import java.util.StringTokenizer;

/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {

    volatile double[] tVec = new double[3];
    volatile double[] eulers = new double[3];

    Thread mainThread = new Thread(() -> {
        try (ZContext context = new ZContext()) {
            ZMQ.Socket subscriber = context.createSocket(SocketType.SUB);
            subscriber.connect("tcp://" + RobotMap.piIp + ":5800");

            String filter = "PNP ";
            subscriber.subscribe(filter.getBytes(ZMQ.CHARSET));

            while (!Thread.interrupted()) {
                String recv = subscriber.recvStr().substring(3);

                SmartDashboard.putString("recv", recv);



                StringTokenizer sscanf = new StringTokenizer(recv, ",");

                tVec[0] = Double.valueOf(sscanf.nextToken());
                tVec[1] = Double.valueOf(sscanf.nextToken());
                tVec[2] = Double.valueOf(sscanf.nextToken());

                eulers[0] = Double.valueOf(sscanf.nextToken());
                eulers[1] = Double.valueOf(sscanf.nextToken());
                eulers[2] = Double.valueOf(sscanf.nextToken());

                Thread.sleep(5);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    });

    public VisionSubsystem() {}

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void start() {
        mainThread.start();
    }

    public void stop() {
        mainThread.interrupt();
    }

    public double[] getTVec() {
        return tVec;
    }

    public double getYaw() {
        return eulers[1];
    }
}
