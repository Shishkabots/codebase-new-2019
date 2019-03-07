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
    double P = 0.005;
    double I = 0;
    double D = 0;
    double integral, previous_error, error, derivative = 0;
    double dt = 0.02;
    double completionThreshold = 2; // also in degrees
    double ff = 0.166;
    
    double maxVoltage = 0.35;

    int itersUnderThreshold = 0;
    int itersComplete = 10;

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
        error = t - gyro.getAngle(); // Error = Target - Actual
        integral += (error * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / dt;
        previous_error = error;
        double voltage = (P * error + I * this.integral + D * derivative);
        voltage += (voltage > 0 ? ff : -ff);
        if(Math.abs(voltage) >= maxVoltage){
            voltage = Math.signum(voltage) * maxVoltage;
        }
        Robot.m_drivetrain.moveWithCurve(0, voltage, true);
        SmartDashboard.putNumber("Gyro Voltage percentage: ", voltage);
        SmartDashboard.putNumber("Gyro Output Angle: ", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Target Angle: ", t);
        SmartDashboard.putNumber("Gyro Integral: ", integral);
        SmartDashboard.putNumber("Gyro Angle Error: ", error);
        SmartDashboard.putNumber("Gyro Derivative: ", derivative);

        if(Math.abs(error) <= completionThreshold){
            itersUnderThreshold++;
        }
        else{
            itersUnderThreshold = 0;
        }
    }

    protected boolean isFinished() {
        return itersUnderThreshold >= itersComplete;
    }
    
    protected void end() {
        Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}