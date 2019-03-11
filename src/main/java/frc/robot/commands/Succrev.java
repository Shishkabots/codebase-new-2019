package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class Succrev extends Command {
    public Succrev() {
        requires(Robot.m_intake);
    }
    //Intake succ = new Intake();
    VictorSPX succrev = Robot.side;
            // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        gtime = 0;
        succrev.set(ControlMode.PercentOutput, -0.2);
    
    }
        
    // Called repeatedly when this Command is scheduled to run
    public int gtime;
    @Override
    protected void execute() {
        gtime++;
        succrev.set(ControlMode.PercentOutput, -0.2);
        
        //succ.set(ControlMode.PercentOutput, Robot.m_oi.succbutt.whileHeld() ? 0.2 : 0)
        // if(gtime > 100) {
        //     succ.set(ControlMode.PercentOutput, 0);
        // }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
        //return gtime >= 220;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        succrev.set(ControlMode.PercentOutput, 0);
        gtime = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        succrev.set(ControlMode.PercentOutput, -0.2);;
    }
}