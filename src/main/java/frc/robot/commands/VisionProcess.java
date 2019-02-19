package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.VisionHelper;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.GripPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileNotFoundException;

public class VisionProcess extends Command {

    public VisionProcess() {
        //requires(Robot.m_hatch);
    }

    UsbCamera cam;
    CvSink sin;
    CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 1280, 720);

            // Called just before this Command runs the first time
    MatOfPoint input = new MatOfPoint();
    GripPipeline grip;
    VisionHelper vhelp;
    double[] x;
    long returnTime;
    @Override
    protected void initialize() {
        cam = Robot.theCamera;
        sin = Robot.cv;
        grip = Robot.pipe;
        vhelp = new VisionHelper();
        returnTime = sin.grabFrame(input);
        outputStream.putFrame(input);
        SmartDashboard.putNumber("Radius: ", -123);
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        
        SmartDashboard.putNumber("Returntime: ", returnTime);

        double robot_offset_x = 0.0;
        double robot_offset_y = 0.0;
        double tape_offset_x = 0.0;
        double tape_offset_y = 0.0;
        double height = 46.0;
        
        SmartDashboard.putNumber("Radius: ", -123);
        // try{
        //     x = vhelp.get_move_to_correct_point(input, robot_offset_x, robot_offset_y, tape_offset_x, tape_offset_y, height);
        // }
        // catch(FileNotFoundException f) {
        //     SmartDashboard.putString("Driver: ", "filenotfound");
        // }
        SmartDashboard.putNumber("Radius: ", -123);
        double[] rThe = vhelp.get_final_R_theta(input, robot_offset_x, robot_offset_y, tape_offset_x, tape_offset_y, height);
        SmartDashboard.putNumber("Radius: ", rThe[0]);
        SmartDashboard.putNumber("Theta: ", rThe[1]);
        new PIDturn(rThe[1]).start();
        new PIDrive(rThe[0]).start();
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true; // only run command once
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}