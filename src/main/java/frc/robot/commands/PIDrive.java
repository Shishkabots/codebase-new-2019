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
    public double diameter = 6; // in inches
    
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDrive(double tt) {
        t = 4096 * tt/(diameter*Math.PI);
        SmartDashboard.putNumber("Running: ", 2);
        //requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        SmartDashboard.putNumber("Running: ", 1);
        Robot.m_drivetrain.move(0, 0);
        Robot.leftTalon.setSelectedSensorPosition(0);
        Robot.rightTalon.setSelectedSensorPosition(0);
        //e1.reset();
    }
    
    protected void execute() {
        SmartDashboard.putNumber("Running: ", 0);
        Robot.leftTalon.follow(Robot.rightTalon);
        SmartDashboard.putNumber("t: ", t);
        Robot.rightTalon.set(ControlMode.Position,2000);
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
    	//Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}