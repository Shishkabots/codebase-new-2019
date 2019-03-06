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
    public double targetTicks;
    public double diameter = 6; // in inches
    
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDrive(double targetInches) {
        targetTicks = 4096 * targetInches/(diameter*Math.PI);
        //SmartDashboard.putNumber("Running: ", 2);
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        //SmartDashboard.putNumber("Running: ", 1);
        Robot.m_drivetrain.move(0, 0);
        Robot.leftTalon.setSelectedSensorPosition(0);
        Robot.rightTalon.setSelectedSensorPosition(0);
    }
    
    protected void execute() {
        //SmartDashboard.putNumber("Running: ", 0);
        SmartDashboard.putNumber("Left Encoder in loop: ", Robot.leftTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right Encoder in loop: ", Robot.rightTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Running: ", 0);
        Robot.leftTalon.set(ControlMode.Position, -1 * targetTicks, DemandType.Neutral, 0);
        SmartDashboard.putNumber("targetTicks: ", targetTicks);
        Robot.rightTalon.set(ControlMode.Position,targetTicks,DemandType.Neutral,0);
    }

    protected boolean isFinished() {
        return Math.abs(Robot.rightTalon.getSelectedSensorPosition(0) - targetTicks) <= 50;

    }
    
    protected void end() {
        Robot.leftTalon.setSelectedSensorPosition(0);
        Robot.rightTalon.setSelectedSensorPosition(0);
        new TeleOpCommands().start();
    	//Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}