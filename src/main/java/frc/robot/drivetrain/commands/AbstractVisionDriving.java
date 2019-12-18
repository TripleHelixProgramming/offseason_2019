/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import static frc.robot.drivetrain.Drivetrain.getDrivetrain;

import com.team2363.controller.PIDController;
import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.drivetrain.Camera;

public abstract class AbstractVisionDriving extends Command {

  private PIDController controller = new PIDController(0.05, 0, 0);
  private Notifier notifier = new Notifier(this::calculate);
  private Camera camera;

  public AbstractVisionDriving() {
    requires(getDrivetrain());
    camera = getDrivetrain().getFrontCamera();
  }

  public abstract double getThrottle();

  @Override
  protected void initialize() {
    notifier.startPeriodic(0.001);
    camera.setDockingMode();
  }

  @Override
  protected void execute() {
    double angleToTarget = camera.getRotationalDegreesToTarget();
    controller.setReference(getDrivetrain().getHeading() + angleToTarget);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    notifier.stop();
    HelixEvents.getInstance().addEvent("DRIVETRAIN", "Stopping Vision Driving");
  }

  @Override
  protected void interrupted() {
    end();
  }

  private void calculate() {
    double output = controller.calculate(getDrivetrain().getHeading());
    getDrivetrain().setVelocityOutput(getThrottle() + output, getThrottle() - output);
  }
}
