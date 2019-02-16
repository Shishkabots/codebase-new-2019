package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchActivate extends Command {
    private int geytime = 0;
    public HatchActivate() {
       geytime = 0;
    }

            // Called just before this Command runs the first time
            
    @Override
    protected void initialize() {
        if(geytime == 0) {
            Robot.ds.set(DoubleSolenoid.Value.kReverse);
        }
        
        SmartDashboard.putString("pogyes?: ", "BEGIN");
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        geytime++;
        if(geytime > 100) {
            Robot.ds.set(DoubleSolenoid.Value.kForward);
            SmartDashboard.putString("pogyes?: ", "pog");
                }            
        SmartDashboard.putNumber("geytime", geytime);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(geytime > 120) {
            SmartDashboard.putString("pogyes?: ", "POG");
            return true;
        }else {
            SmartDashboard.putString("pogyes?: ", "CONTINUE");
            return false;
        }
    
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.ds.set(DoubleSolenoid.Value.kForward);
        Robot.ds.set(DoubleSolenoid.Value.kOff);
        geytime = 0;
        SmartDashboard.putNumber("geytime", 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	Robot.m_hatch.setState("Off");
    }
}