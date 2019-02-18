package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.AnalogGyro;

import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class DriveTrainControl extends Command {

    //m_drivetrain is a drivetrain subsystem btw
    public DriveTrainControl() {
        requires(Robot.m_drivetrain);
    }
    Encoder e1 = Robot.e1;
    Encoder e2 = Robot.e2;
    AHRS gyro = Robot.gyro;


    protected void initialize() {
    		Robot.m_drivetrain.move(0, 0);
    }

    protected void execute() {
        double speed = Robot.m_oi.boost.get() ? 1.0 : .5;
        double lTrigger = Robot.m_oi.controllerOne.getRawAxis(2);
        double rTrigger = Robot.m_oi.controllerOne.getRawAxis(3);
        double turnAxis = Robot.m_oi.controllerOne.getRawAxis(0);
        
        //3 is right trigger, 2 is left trigger, 0 is x axis of left stick, unsure of math
    	Robot.m_drivetrain.moveWithCurve(
            (rTrigger - lTrigger) * speed,
            turnAxis * 0.5 * (rTrigger > lTrigger ? 1 : -1),
            true);
        
            
        //SmartDashboard.putNumber("Encoder 1: ", e1.getDistance());
        //SmartDashboard.putNumber("Encoder 2: ", e2.getDistance());
        SmartDashboard.putNumber("leftEncoder Pos", Robot.leftTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RightEncoder Pos", Robot.rightTalon.getSelectedSensorPosition(1));
        SmartDashboard.putNumber("Gyro Output Angle: ", gyro.getAngle());

        //alternative drive mode, can't go backwards
        //Robot.m_drivetrain.arcadeDrive(Robot.m_oi.controllerOne.getRawAxis(3), Robot.m_oi.controllerOne.getRawAxis(0)* 0.5);
        
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	Robot.m_drivetrain.move(0, 0);
    }
}