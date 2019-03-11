package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 *
 */
public class CargoIntake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public WPI_VictorSPX intake = Robot.side;

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void spin(double voltage){
        intake.set(ControlMode.PercentOutput, voltage);
    }
}