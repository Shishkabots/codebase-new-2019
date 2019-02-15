package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class Succ extends Command {
    public Succ() {
        requires(Robot.m_intake);
    }
    Intake succ = new Intake();
            // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        succ.Succ("ON");
    
    }
        
    // Called repeatedly when this Command is scheduled to run
    public int geytime;
    @Override
    protected void execute() {
        geytime++;
        if(geytime > 200) {
            succ.Succ("OFF");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return geytime >= 220;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        succ.Succ("OFF");
        geytime = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        succ.Succ("OFF");
    }
}