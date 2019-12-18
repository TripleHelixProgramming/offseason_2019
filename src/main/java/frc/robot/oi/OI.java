/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import static com.team2363.utilities.ControllerMap.X_BOX_LEFT_STICK_Y;
import static com.team2363.utilities.ControllerMap.X_BOX_RIGHT_STICK_X;
import static com.team2363.utilities.ControllerPatroller.getPatroller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.drivetrain.commands.RampDown;
import frc.robot.drivetrain.commands.VisionTakeOverGroup;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI INSTANCE;

  /**
   * @return retrieves the singleton instance of the Operator Interface
   */
  public static OI getOI() {
    if (INSTANCE == null) {
      INSTANCE = new OI();
    }
    return INSTANCE;
  }

  private final String DRIVER = "Xbox";
  private final int DRIVER_PORT = 0;
  private final String OPERATOR = "P4";
  private final int OPERATOR_PORT = 1;

  private Joystick driver = getPatroller().get(DRIVER, DRIVER_PORT);
  private Joystick operator = getPatroller().get(OPERATOR, OPERATOR_PORT);

  private OI() { 
    new JoystickButton(driver, 3).whenPressed(new RampDown(3, 0));
    new JoystickButton(driver, 2).whenPressed(new VisionTakeOverGroup());
  }

  /**
   * @return the raw controller throttle
   */
  public double getThrottle() {
    return driver.getRawAxis(X_BOX_LEFT_STICK_Y); 
	}
  
  /**
   * @return the raw controller turn
   */
  public double getTurn() {
    return driver.getRawAxis(X_BOX_RIGHT_STICK_X);
  }
  
  /**
	 * Turns on and off the rumble function on the driver and operator controllers
	 * @param set true to turn on rumble
	 */
	public void setControllerRumble(boolean rumble) {
		if (rumble) {
			setRumble(driver, 1);
			setRumble(operator, 1);
		} else {
			setRumble(driver, 0);
			setRumble(operator, 0);
		}
  }
  
  private void setRumble(Joystick controller, int state) {
    controller.setRumble(RumbleType.kLeftRumble, state);
		controller.setRumble(RumbleType.kRightRumble, state);
  }
}