package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchLever extends Command {
    private int gtime = 0;
    private final int endtime = 10;
    
    public HatchLever() {
       gtime = 0;
    }

            // Called just before this Command runs the first time
            
    @Override
    protected void initialize() {
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        double povAngle = Robot.m_oi.controllerTwo.getPOV();
        WPI_VictorSPX vic = Robot.intake;
        double voltage = 1.0;
        if(povAngle != -1){
            if(povAngle > 90 && povAngle < 270){ // any of the bottom three (down, down right, down left)
                vic.set(ControlMode.PercentOutput, -voltage);
                // make new command to run -0.5 or some voltage for a certain amount of time (no need to hold)
            }
            else if(povAngle < 90 || povAngle > 270){ // any of the top three (up, up right, up left)
                vic.set(ControlMode.PercentOutput, voltage);
            }
        }
        SmartDashboard.putNumber("povAngle", povAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        //Robot.ds.set(DoubleSolenoid.Value.kForward);
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        //Robot.m_hatch.setState("Off");
    }
}