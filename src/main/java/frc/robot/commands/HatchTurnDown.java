package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.HatchLever;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class HatchTurnDown extends Command {
    private int timecounter = 0;
    private final int endtime = 10;
    public HatchTurnDown() {
        requires(Robot.m_lever);
        timecounter = 0;
    }
    HatchLever intake = Robot.m_lever;
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        intake.turn(0);
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        timecounter++;
        //double speedCoef = 0.7;
        //SmartDashboard.putNumber("ltrig", lTrigger);
        //SmartDashboard.putNumber("rtrig", rTrigger);
        //voltage at which motor controlling intake should be spun at
        intake.turn(-0.3);    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(timecounter > endtime) {
            //SmartDashboard.putString("Hatch Piston: ", "Done");
            return true;
        }else {
            //SmartDashboard.putString("Hatch Piston: ", "Not Done");
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        intake.turn(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        intake.turn(0);
    }
}