package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PIDall extends CommandGroup {
 
  // distance/tapeforward will be negative, since we want robot to go backwards
  public PIDall(double angleDegrees, double distance, double tapeAngle, double tapeForward) {
    addSequential(new PIDturn(angleDegrees));
    addSequential(new PIDrive(distance));
    addSequential(new PIDturn(tapeAngle));
    addSequential(new PIDrive(tapeForward));
  }
}
