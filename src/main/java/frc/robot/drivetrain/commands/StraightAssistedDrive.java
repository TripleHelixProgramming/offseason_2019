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

import java.util.Date;

import com.team2363.commands.NormalizedArcadeDrive;
import com.team2363.controller.PIDController;
import com.team2363.utilities.RollingAverager;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.Drivetrain.CommandUnits;

/**
 * This command will check to see if there is no turn command being applied and will attempt to 
 * hold the current heading
 */
public class StraightAssistedDrive extends NormalizedArcadeDrive {

  private PIDController controller = new PIDController(0.001, 0, 0, 0.02);
  private boolean holdingHeading;
  private Date turnTimeout;
  
  private RollingAverager throttleAverage = new RollingAverager(20);

  public StraightAssistedDrive() {
    super(getDrivetrain());
    controller.setOutputRange(-1, 1);
  }

  @Override
  protected double getThrottle() {
    double throttle = getOI().getThrottle();
    if (Math.abs(throttle) < 0.05) {
      throttle = 0;
    }
    return throttleAverage.getNewAverage(throttle);
  }

  @Override
  protected double getTurn() {
    double turn =  getOI().getTurn();
    double turnPower = abs(turn);
    double currentHeading = getDrivetrain().getHeading();

    // Is the robot being commanded to turn? If yes then use that as the command and reset the hold heading
    if (turnPower > 0.05) {
      holdingHeading = false;
      turnTimeout = null;
      return turn;
    }

    if (turnTimeout == null) {
      turnTimeout = new Date();
    }

    if (!holdingHeading && new Date().getTime() - turnTimeout.getTime() < 500) {
      return 0;
    }

    // Set the hold heading if this is the first time we see no turn command
    if (!holdingHeading) {
      holdingHeading = true;
      controller.reset();
      controller.setReference(currentHeading);
    }

    // Return the P controlled error as our turn value to keep our current heading
    return -controller.calculate(currentHeading);
  }

  @Override
  protected void useOutputs(double left, double right) {
    getDrivetrain().setSetpoint(CommandUnits.PERCENT_FULLSPEED, left, right);
  }

  @Override
  protected void execute() {
    super.execute();
    SmartDashboard.putBoolean("Holding Heading", holdingHeading);
    SmartDashboard.putNumber("Hold Heading", controller.getReference());
    SmartDashboard.putNumber("Current Heading", getDrivetrain().getHeading());
    SmartDashboard.putNumber("Current Turn Command", getOI().getTurn());
  }
}
