package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.AnalogGyro;

/**
 *
 */
public class PIDturn extends Command {
    public AnalogGyro g = Robot.gyro;
    public double t;
    int P, I, D = 1;
    double integral, previous_error, error, derivative = 0;
    double rcw;
    double dt = 0.02;
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDturn(double tt) {
        t = tt;
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
    	Robot.m_drivetrain.move(0, 0);
    }
    
    protected void execute() {
        error = t - g.getAngle(); // Error = Target - Actual
        integral += (error * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / dt;
        Robot.m_drivetrain.arcadeDrive(0,P*error + I*this.integral + D*derivative);
    }

    protected boolean isFinished() {
        return (Math.abs(error) <= 0.5);
    }
    
    protected void end() {
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}