package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class Hatch extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public DoubleSolenoid Piston1 = Robot.ds;
    


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setState(String state){
        if(state == "Off"){
            Robot.ds.set(DoubleSolenoid.Value.kOff);
            
        }
        else if(state == "Close"){
            Robot.ds.set(DoubleSolenoid.Value.kForward);
            
        }
        else if(state == "Open"){
            Robot.ds.set(DoubleSolenoid.Value.kReverse);
            
        }
    }
}