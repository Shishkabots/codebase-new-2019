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

import java.io.FileNotFoundException;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionProcess extends Command {


    UsbCamera cam;
    CvSink sin;
    //CvSource outputStream;

            // Called just before this Command runs the first time
    Mat input = new Mat();
    Mat output = new Mat();
    GripPipeline grip;
    VisionHelper vhelp = new VisionHelper();
    double[] x;
    long returnTime;
    public VisionProcess() {
        //requires(Robot.m_hatch);
        
    }

    
   
    
    @Override
    protected void initialize() {
        cam = Robot.theCamera;
        grip = Robot.pipe;
        sin = Robot.cv;
        //outputStream = CameraServer.getInstance().putVideo("Blur",1280,720);

        SmartDashboard.putNumber("Start: ", 1);
        
        sin.grabFrame(input,20000);
        
        SmartDashboard.putNumber("img width", input.width());
        SmartDashboard.putNumber("img length", input.height());
        
        // SmartDashboard.putNumber("numero uno:", input.get(640, 360)[0]);
        // SmartDashboard.putNumber("numero rwo:", input.get(640, 360)[1]);
        // SmartDashboard.putNumber("numero rsan:", input.get(640, 360)[2]);
        // SmartDashboard.putNumber("numero duno:", input.get(123, 214)[0]);
        // SmartDashboard.putNumber("numero drwo:", input.get(123, 214)[1]);
        // SmartDashboard.putNumber("numero drsan:", input.get(123, 214)[2]);
        
       // Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2GRAY);
        //outputStream.putFrame(input);
        SmartDashboard.putNumber("End VP Init: ", 1);
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {

        
        //SmartDashboard.putNumber("Returntime: ", returnTime);

        // tune these values
        double robot_offset_x = 0.0;
        double robot_offset_y = 0.0;
        double tape_offset_x = 0.0;
        double tape_offset_y = 8.5;
        double height = 46.0;
        
        //SmartDashboard.putString("Driver: ", "file is found");
        if(input == null){
            SmartDashboard.putString("Input img", "None found");
        }
        else{
            SmartDashboard.putString("Input img", "Loaded");
        }

        // try{
        //     x = vhelp.get_move_to_correct_point(input, robot_offset_x, robot_offset_y, tape_offset_x, tape_offset_y, height);
        // }
        // catch(FileNotFoundException f) {
        //     SmartDashboard.putString("Driver: ", "filenotfound");
        // }
        // SmartDashboard.putNumber("Radius: ", -123);
        // SmartDashboard.putNumber("Radius: ", x[0]);
        // SmartDashboard.putNumber("Theta: ", x[1]);
        // new PIDturn(x[1]).start();
        // new PIDrive(x[0]).start();

        double[] rThe = vhelp.get_final_R_theta(input, robot_offset_x, robot_offset_y, tape_offset_x, tape_offset_y, height);
        if(rThe[0] == -1 && rThe[1] == -1){
            SmartDashboard.putString("Successful Macro", "No");
            SmartDashboard.putNumber("VHelp Done:", 1);
            SmartDashboard.putNumber("Radius: (inches)", -1);
            SmartDashboard.putNumber("Theta: (degrees)", -1);
        }
        else{
            SmartDashboard.putString("Successful Macro", "Yes");
            
            SmartDashboard.putNumber("VHelp Done:", 1);
            SmartDashboard.putNumber("Radius: (inches)", rThe[0]);
            SmartDashboard.putNumber("Theta: (degrees)", rThe[1] * 180.0 / Math.PI);
            //new PIDturn(rThe[1]).start();
            //new PIDrive(rThe[0]).start();
        }
        
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        SmartDashboard.putNumber("VP Done", 1);
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}