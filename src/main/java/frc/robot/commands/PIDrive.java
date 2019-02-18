package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDrive extends Command {
    //public Encoder e1 = Robot.e1;
    //public Encoder e2 = Robot.e2;
    public double t;
    
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDrive(double tt) {
        t = 4096 * tt/(6*3.14);
        //requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        //e1.reset();
    }
    
    protected void execute() {
        Robot.leftTalon.follow(Robot.rightTalon);
        Robot.rightTalon.set(ControlMode.Position,t,DemandType.Neutral,0);
    }

    protected boolean isFinished() {
        return true;
    }
    
    protected void end() {
    	//Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}