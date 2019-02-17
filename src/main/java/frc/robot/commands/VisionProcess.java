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

            // Called just before this Command runs the first time
    MatOfPoint input;
    GripPipeline grip;
    VisionHelper vhelp;
    double[] x;
    @Override
    protected void initialize() {
        cam = Robot.theCamera;
        sin = Robot.cv;
        grip = Robot.pipe;
        vhelp = new VisionHelper();
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        sin.grabFrame(input);
        try{
            x = vhelp.get_move_to_correct_point(input, 0.0, 0.0, 0.0 ,0.0 ,46.0);
        }
        catch(FileNotFoundException f) {
            
        }
        new PIDturn(x[1]).start();
        new PIDrive(x[0]).start();
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
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