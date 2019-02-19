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
  public Joystick controllerOne = new Joystick(0);
	public Button boost = new JoystickButton(controllerOne,6);

  public Joystick controllerTwo = new Joystick(1);//also an controllerOne
  
  public Button hatchbutt = new JoystickButton(controllerOne,3);
  //public Button hbutt = new JoystickButton(controllerTwo, 2);
  public Button succbutt = new JoystickButton(controllerTwo, 1);       
  public Button succbuttrev = new JoystickButton(controllerTwo, 2);    
  public Button camerabutt = new JoystickButton(controllerTwo,3);
  //public Button gyroButton = new JoystickButton(controllerTwo, 3);     
  public Button press = new JoystickButton(controllerTwo, 4);         
  public Button reverse = new JoystickButton(controllerOne, 9);
  public OI() {
    hatchbutt.whenPressed(new HatchActivate());
    //succbutt.whenPressed(new Succ());
    succbutt.whileHeld(new Succ());
    //succbuttrev.whenPressed(new Succrev());
    succbuttrev.whileHeld(new Succrev());
    camerabutt.whenPressed(new VisionProcess());
    //gyroButton.whenPressed(new PIDturn(-45));
    //press.whenPressed(new PIDrive(20));
    press.whenPressed(new VisionProcess());
    reverse.whenPressed(new Reverse());
  }
}
