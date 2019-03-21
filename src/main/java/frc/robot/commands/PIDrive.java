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
    public double targetTicks;
    public double diameter = 6; // in inches
    
    double P = 0.00007;
    double I = 0;
    double D = 0;
    double integral, previous_error, error, derivative = 0;
    double dt = 0.02;
    // max estimated maybe 10 feet per second?
    // 1 inch per second is roughly 20 ticks per 100 ms
    double completionThreshold = 100; // in ticks (each tick is 0.005 inches roughly, with circumference = ~19 inches)
    double ff = 0.1;

    double maxVoltage = 0.35 + ff;

    int itersUnderThreshold = 0;
    int itersComplete = 20;
    
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
        SmartDashboard.putString("RAN: ", "No");
        //Robot.leftTalon.set(ControlMode.Velocity, -300); // using voltage output for now but it should be velocity
        //Robot.rightTalon.set(ControlMode.Velocity, 300);
    }
    
    protected void execute() {
        //SmartDashboard.putNumber("Running: ", 0);
        double leftTicks = Robot.leftTalon.getSelectedSensorPosition(0); // adjusted for the encoder flipped on left
        double rightTicks = -Robot.rightTalon.getSelectedSensorPosition(0);
        double avgTicks = (leftTicks + rightTicks) / 2;
        error = targetTicks - avgTicks; // Error = Target - Actual
        integral += (error * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / dt;
        previous_error = error;

        double voltage = (P * error + I * this.integral + D * derivative);
        voltage += (voltage > 0 ? ff : -ff);
        if(Math.abs(voltage) >= maxVoltage){
            voltage = Math.signum(voltage) * maxVoltage;
        }
        Robot.m_drivetrain.moveWithCurve(voltage, 0, true);

        //Robot.leftTalon.set(ControlMode.PercentOutput, 0.20);
        //Robot.rightTalon.set(ControlMode.PercentOutput, 0.20);
        //Robot.leftTalon.set(ControlMode.Velocity, 4096/600 * 250); // using voltage output for now but it should be velocity
        //Robot.rightTalon.set(ControlMode.Velocity, 4096/600 * 250);
        // SmartDashboard.putNumber("left velocity target:", Robot.leftTalon.getClosedLoopTarget());
        // SmartDashboard.putNumber("left velocity error:", Robot.leftTalon.getClosedLoopError());
        // SmartDashboard.putNumber("left velocity current:", Robot.leftTalon.getSelectedSensorVelocity());

        // SmartDashboard.putNumber("right velocity target:", Robot.rightTalon.getClosedLoopTarget());
        // SmartDashboard.putNumber("right velocity error:", Robot.rightTalon.getClosedLoopError());
        // SmartDashboard.putNumber("right velocity current:", Robot.rightTalon.getSelectedSensorVelocity());
        

        SmartDashboard.putNumber("Encoder Voltage percentage: ", voltage);
        SmartDashboard.putNumber("Left Encoder Ticks: ", leftTicks);
        SmartDashboard.putNumber("Right Encoder Ticks: ", rightTicks);
        SmartDashboard.putNumber("Average Ticks: ", avgTicks);
        SmartDashboard.putNumber("Target Ticks: ", targetTicks);
        SmartDashboard.putNumber("Encoder Error Integral: ", integral);
        SmartDashboard.putNumber("Encoder Error: ", error);
        SmartDashboard.putNumber("Encoder Error Derivative: ", derivative);

        // Robot.leftTalon.set(ControlMode.Velocity, -150); // using voltage output for now but it should be velocity
        // Robot.rightTalon.set(ControlMode.Velocity, 150);
        // SmartDashboard.putNumber("Velocity1:", Robot.rightTalon.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Velocity2:", Robot.leftTalon.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Pos:", Robot.rightTalon.getSelectedSensorPosition());

        if(Math.abs(error) <= completionThreshold){
            itersUnderThreshold++;
        }
        else{
            itersUnderThreshold = 0;
        }

        
        //SmartDashboard.putNumber("Running: ", 0);
        //Robot.leftTalon.set(ControlMode.Position, -1 * targetTicks, DemandType.Neutral, 0);
        //Robot.rightTalon.set(ControlMode.Position, targetTicks, DemandType.Neutral,0);
    }

    protected boolean isFinished() {
        //SmartDashboard.putString("RAN: ", "Run");
        return itersUnderThreshold >= itersComplete;
        //return Math.abs(Math.abs(Robot.leftTalon.getSelectedSensorPosition()) - (4096*2)) <= 300;
        //return Robot.rightTalon.getSelectedSensorVelocity() >= 1000;
    }
    
    protected void end() {
        Robot.leftTalon.setSelectedSensorPosition(0);
        Robot.rightTalon.setSelectedSensorPosition(0);
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}