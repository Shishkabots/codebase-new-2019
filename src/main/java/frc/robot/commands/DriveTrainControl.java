package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class DriveTrainControl extends Command {

    //m_drivetrain is a drivetrain subsystem btw
    public DriveTrainControl() {
        requires(Robot.m_drivetrain);
    }

    protected void initialize() {
    		Robot.m_drivetrain.move(0, 0);
    }

    protected void execute() {
        double speed = Robot.m_oi.boost.get() ? 1.0 : .5;
        double lTrigger = Robot.m_oi.xbox.getRawAxis(2);
        double rTrigger = Robot.m_oi.xbox.getRawAxis(3);
        double turnAxis = Robot.m_oi.xbox.getRawAxis(0);
        
        //3 is right trigger, 2 is left trigger, 0 is x axis of left stick, unsure of math
    	Robot.m_drivetrain.moveWithCurve(
            (rTrigger - lTrigger) * speed,
            turnAxis * 0.5 * (rTrigger > lTrigger ? 1 : -1),
            true);
        
        //alternative drive mode, can't go backwards
        //Robot.m_drivetrain.arcadeDrive(Robot.m_oi.xbox.getRawAxis(3), Robot.m_oi.xbox.getRawAxis(0)* 0.5);
        
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}