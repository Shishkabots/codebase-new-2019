package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDall extends CommandGroup {
 
  // distance/tapeforward will be negative, since we want robot to go backwards
  // public PIDall(double angleDegrees, double distance, double tapeAngle, double tapeForward) {
  //   addSequential(new PIDturn(angleDegrees));
  //   addSequential(new PIDrive(distance));
  //   addSequential(new PIDturn(tapeAngle));
  //   addSequential(new PIDrive(tapeForward));
  // }

  public PIDall(double theta1, double Dy, boolean clockwise, double Dx) {
    addSequential(new PIDturn(theta1));

    addSequential(new PIDrive(Dy));
    if(clockwise){
      addSequential(new PIDturn(90));
      SmartDashboard.putNumber("Turn2: (degrees)", 90);
    }
    else{
      addSequential(new PIDturn(-90));
      SmartDashboard.putNumber("Turn2: (degrees)", -90);
    }
    //addSequential(new PIDrive(Dx));

    SmartDashboard.putNumber("Turn1: (degrees)", theta1);
    SmartDashboard.putNumber("Dy, made negative: (inches)", Dy);
    SmartDashboard.putNumber("Dx, made negative: (inches)", Dx);
    
    addSequential(new TeleOpCommands());
  }
}
