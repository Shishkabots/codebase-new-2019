package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class Succ extends Command {
    public Succ() {
        requires(Robot.m_intake);
    }
    //Intake succ = new Intake();
    VictorSPX succ = Robot.side;
            // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        gtime = 0;
        succ.set(ControlMode.PercentOutput, .2);
    
    }
        
    // Called repeatedly when this Command is scheduled to run
    public int gtime;
    @Override
    protected void execute() {
        gtime++;
        if(gtime > 100) {
            succ.set(ControlMode.PercentOutput, 0);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return gtime >= 220;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        succ.set(ControlMode.PercentOutput, 0);
        gtime = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        succ.set(ControlMode.PercentOutput, .2);;
    }
}