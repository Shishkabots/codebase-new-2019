package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 */
public class PIDrive extends Command {
    public Encoder e1 = Robot.e1;
    public Encoder e2 = Robot.e2;
    public double t;
    int P, I, D = 1;
    double integral, previous_error, error, derivative = 0;
    double rcw;
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDrive(double tt) {
        t = tt;
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
    	Robot.m_drivetrain.move(0, 0);
    }
    
    protected void execute() {
        error = t - e1.getDistance(); // Error = Target - Actual
        integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / .02;
        Robot.m_drivetrain.arcadeDrive(P*error + I*this.integral + D*derivative,0);
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