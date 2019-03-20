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
        requires(Robot.m_drivetrain);
    }

    
   
    
    @Override
    protected void initialize() {
        cam = Robot.theCamera;
        grip = Robot.pipe;
        if(Robot.testing){
            SmartDashboard.putNumber("Start: ", 1);
            SmartDashboard.putNumber("img width", input.width());
            SmartDashboard.putNumber("img length", input.height());
        }
       // Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2GRAY);
        //outputStream.putFrame(input);
        //SmartDashboard.putNumber("End VP Init: ", 1);
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {

        sin = Robot.cv;
        //outputStream = CameraServer.getInstance().putVideo("Blur",1280,720);

        if(!Robot.t.interrupted()) {
            sin.grabFrame(input,20000);
        }else if(Robot.testing) {
            SmartDashboard.putString("??????: ", "I nothing am");
        }

        // tune these values (it's relative to center of turning, which is not quite physical wheel center)
        double robot_offset_x = 0.0;
        double robot_offset_y = 14.0; // 10.75 is calibrated value as of the half-cargo intake robot state
        // setting tape offset to be 0 makes problems for the theta computation for 2nd turn, make sure they aren't both 0.
        double tape_offset_x = 8.0;
        double tape_offset_y = -1.8; // there's not supposed to be any but we're doing the marketing video lmao
        // in case we need to move forward after the second turn (i.e. aligned with tape)
        // since if we go to that final point in the first place, we might hit something when we turn second time
        // this value is the forward distance from the CENTER of the tape
        double tape_center_final_offset = 0.0;
        double height = 46.7;
        
        //SmartDashboard.putString("Driver: ", "file is found");
        if(Robot.testing && input == null){
            //SmartDashboard.putString("Input img", "None found");
        }
        else if(Robot.testing) {
            //SmartDashboard.putString("Input img", "Loaded");
        }


        //double[] rTheta = vhelp.get_final_R_theta(input, robot_offset_x, robot_offset_y, tape_offset_x, tape_offset_y, height);
        //double[] rThetaNoTapeOffset = vhelp.get_final_R_theta(input, robot_offset_x, robot_offset_y, 0, 0, height);
        double[] rTheta = vhelp.newGetThetaAndR(input, robot_offset_y, tape_offset_x, tape_offset_y, height);

        if(rTheta[0] == -1 && rTheta[1] == -1 && rTheta[2] == -1){
            SmartDashboard.putString("Successful Macro", "No");
            //SmartDashboard.putNumber("VHelp Done:", 1);
            SmartDashboard.putNumber("Radius: (inches)", -1);
            SmartDashboard.putNumber("Theta: (degrees)", -1);

            new TeleOpCommands().start();
        }
        else{
            SmartDashboard.putString("Successful Macro", "Yes");
            
            //SmartDashboard.putNumber("VHelp Done:", 1);
            // SmartDashboard.putNumber("Radius: (inches)", rTheta[0]);
            // SmartDashboard.putNumber("Theta: (degrees)", rTheta[1] * 180.0 / Math.PI);

            SmartDashboard.putNumber("Theta: (degrees)", Math.toDegrees(rTheta[0]));
            SmartDashboard.putNumber("Dy: (inches)", rTheta[1]);
            SmartDashboard.putNumber("Dx: (inches)", rTheta[2]);
            
            // make the distances negative since we want robot to be moving backwards
            // tape offset being 0 is bad since we don't know angle, don't make tape offset 0 please
            // if(tape_offset_x == 0.0 && tape_offset_y == 0.0){
            //     new PIDall(rTheta[1] * 180.0 / Math.PI, -rTheta[0], 0, 0).start();
            // }
            // else{
            //     //new PIDall(rTheta[1] * 180.0 / Math.PI, -rTheta[0], secondRTheta[1] * 180.0 / Math.PI, -(secondRTheta[0]+tape_center_final_offset)).start(); // PASS IN A NEGATIVE, SINCE WE WANT TO DRIVE BACKWARDS (camera on back of robot)
            //     new PIDall(rTheta[1] * 180.0 / Math.PI, -rTheta[0], 0, 0).start(); // PASS IN A NEGATIVE, SINCE WE WANT TO DRIVE BACKWARDS (camera on back of robot)
            // }

            double theta1 = rTheta[0];
            double theta3 = rTheta[3];
            double Dy = rTheta[1];
            double Dx = rTheta[2];
            //new PIDall(Math.toDegrees(theta1), -Dy, (theta3 - theta1) > 0, -Dx).start();
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
        //SmartDashboard.putNumber("VP Done", 1);
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}