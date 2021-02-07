/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.Drivetrain.CommandUnits;

public class RampDown extends Command {

  private double heading;
  private double acceleration;
  private double currentVelocity;
  private double distance;
  private double kP;
  private int directionScalar;

  public RampDown(double distance, double kP) {
    requires(Drivetrain.getDrivetrain());
    this.distance = distance;
    this.kP = kP;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    heading = Drivetrain.getDrivetrain().getHeading();
    double initialVelocity = (Drivetrain.getDrivetrain().getLeftVelocity() + Drivetrain.getDrivetrain().getRightVelocity()) / 2.0;
    currentVelocity = initialVelocity;

    if (initialVelocity < 0) {
      directionScalar = -1;
    } else {
      directionScalar = 1;
    }

    double deceleratrionTime = (2.0 * distance) / initialVelocity;
    acceleration = initialVelocity / deceleratrionTime;
    acceleration *= 0.02;
    acceleration *= directionScalar;
    HelixEvents.getInstance().addEvent("DRIVETRAIN", "Starting Rampdown");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double headingError = heading - Drivetrain.getDrivetrain().getHeading();
    double headingCorrection = headingError * kP;
    Drivetrain.getDrivetrain().setSetpoint(CommandUnits.FPS, currentVelocity + headingCorrection, currentVelocity - headingCorrection);
    currentVelocity -= acceleration;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (directionScalar < 0) {
      return currentVelocity >= 0;
    } else {
      return currentVelocity <= 0;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drivetrain.getDrivetrain().setSetpoint(CommandUnits.FPS, 0, 0);
    HelixEvents.getInstance().addEvent("DRIVETRAIN", "Finished Rampdown");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
