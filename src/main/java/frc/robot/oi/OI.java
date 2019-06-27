/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import static com.team2363.utilities.ControllerMap.X_BOX_LEFT_STICK_Y;
import static com.team2363.utilities.ControllerMap.X_BOX_RIGHT_STICK_X;
import static frc.robot.ControllerPatroller.getPatroller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

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

  private OI() {
  }

  /**
   * @return the raw controller throttle
   */
  public double getThrottle() {
    return getPatroller().get(DRIVER, DRIVER_PORT).getRawAxis(X_BOX_LEFT_STICK_Y); 
	}
  
  /**
   * @return the raw controller turn
   */
  public double getTurn() {
    return getPatroller().get(DRIVER, DRIVER_PORT).getRawAxis(X_BOX_RIGHT_STICK_X);
  }
  // public double getGMPOV() {
  //   return operator.getPOV();
  // }
  
  /**
	 * Turns on and off the rumble function on the driver and operator controllers
	 * @param set true to turn on rumble
	 */
	public void setControllerRumble(boolean state) {
		if (state == true) {
			getPatroller().get(DRIVER, DRIVER_PORT).setRumble(RumbleType.kLeftRumble, 1);
			getPatroller().get(DRIVER, DRIVER_PORT).setRumble(RumbleType.kRightRumble, 1);  
			getPatroller().get(OPERATOR, OPERATOR_PORT).setRumble(RumbleType.kLeftRumble, 1);
			getPatroller().get(OPERATOR, OPERATOR_PORT).setRumble(RumbleType.kRightRumble, 1);
		} else {
			getPatroller().get(DRIVER, DRIVER_PORT).setRumble(RumbleType.kLeftRumble, 0);
			getPatroller().get(DRIVER, DRIVER_PORT).setRumble(RumbleType.kRightRumble, 0);
			getPatroller().get(OPERATOR, OPERATOR_PORT).setRumble(RumbleType.kLeftRumble, 0);
			getPatroller().get(OPERATOR, OPERATOR_PORT).setRumble(RumbleType.kRightRumble, 0);
		}
	}
}