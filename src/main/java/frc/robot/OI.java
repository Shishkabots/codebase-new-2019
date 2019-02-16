/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick xbox = new Joystick(0);
	public Button boost = new JoystickButton(xbox,6);

  public Joystick joystick = new Joystick(1);//also an xbox
  
  public Button hatchbutt = new JoystickButton(xbox,3);
  //public Button hbutt = new JoystickButton(joystick, 2);
	public Button succbutt = new JoystickButton(joystick, 6);
  public Button camerabutt = new JoystickButton(joystick,5);
  
  public OI() {
    hatchbutt.whenPressed(new HatchActivate());
    succbutt.whenReleased(new Succ());
  }
}
