// not being used; this is not the vision pipeline

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import org.opencv.core.*;
import org.opencv.core.Point;

public class FollowCamera extends Command {
    public FollowCamera() {
        //requires(Robot.m_hatch);
    }

    private final Object img = Robot.imgLock;
    public static double centerX;
    public static RotatedRect rec;
    public static Point p1;
    public static Point p2;
    public static double eq;
    public static double eq2;

            // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        synchronized(img) {
            centerX = Robot.m_centerX;
            rec = Robot.r;
            //p1 = rec.tl();
            //p2 = rec.br();
            //eq = rec.y/rec.x;
            eq2 = p1.y/p2.x;
        }
        while(!(centerX > 150 && centerX < 170)) {
            double turn = centerX - 160;
            Robot.m_drivetrain.moveWithCurve(-0.5, turn*0.005, false);
        }
        
        if(centerX > 150 && centerX < 170) {
            while(eq2 != eq) {
                Robot.m_drivetrain.moveWithCurve(0.005,(eq2-eq)*0.005,true);
            }
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
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}