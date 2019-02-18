package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDturn extends Command {
    public AHRS gyro = Robot.gyro; // angles are in degrees
    public double t; // target
    double P = 1;
    double I = 0;
    double D = 0;
    double integral, previous_error, error, derivative = 0;
    double rcw;
    double dt = 0.02;
    double completionThreshold = 0.5;
    
    //m_drivetrain is a drivetrain subsystem btw
    public PIDturn(double tt) {
        t = tt;
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        gyro.zeroYaw();
    }
    
    protected void execute() {
        error = (1 / 180) * (t - gyro.getAngle()); // Error = Target - Actual
        integral += (error * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / dt;
        //Robot.m_drivetrain.moveWithCurve(0, P * error + I * this.integral + D * derivative, true);
        SmartDashboard.putNumber("Gyro Output Angle: ", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Integral: ", integral);
        SmartDashboard.putNumber("Gyro Error: ", error);
        SmartDashboard.putNumber("Gyro Derivative: ", derivative);
    }

    protected boolean isFinished() {
        return (Math.abs(error) <= completionThreshold);
    }
    
    protected void end() {
        Robot.m_drivetrain.move(0, 0);
        new TeleOpCommands().start();
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}