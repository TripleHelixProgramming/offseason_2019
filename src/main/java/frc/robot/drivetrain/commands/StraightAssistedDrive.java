/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import static frc.robot.drivetrain.Drivetrain.getDrivetrain;
import static frc.robot.oi.OI.getOI;
import static java.lang.Math.abs;
import static frc.robot.drivetrain.HelixMath.constrainToRange;

import com.team2363.commands.NormalizedArcadeDrive;

/**
 * This command will check to see if there is no turn command being applied and will attempt to 
 * hold the current heading
 */
public class StraightAssistedDrive extends NormalizedArcadeDrive {

  private Double holdHeading = null;
  private double kP = 0.1;

  public StraightAssistedDrive() {
    super(getDrivetrain());
  }

  @Override
  protected double getThrottle() {
    return getOI().getThrottle();
  }

  @Override
  protected double getTurn() {
    double turn =  getOI().getTurn();
    double turnPower = abs(turn);
    double currentHeading = getDrivetrain().getYaw();

    // Is the robot being commanded to turn? If yes then use that as the command and reset the hold heading
    if (turnPower > 0.05) {
      holdHeading = null;
      return turn;
    }

    // Set the hold heading if this is the first time we see no turn command
    if (holdHeading == null) {
      holdHeading = currentHeading;
    }

    // Return the P controlled error as our turn value to keep our current heading
    return constrainToRange(-1, 1, (currentHeading - holdHeading) * kP);
  }

  @Override
  protected void useOutputs(double left, double right) {
    getDrivetrain().tankDrive(left, right);
  }
}
