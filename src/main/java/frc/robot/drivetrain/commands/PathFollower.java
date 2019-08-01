/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import static frc.robot.drivetrain.Drivetrain.getDrivetrain;

import java.io.IOException;

import com.team2363.controller.PIDController;
import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public abstract class PathFollower extends Command {
  // Used for correcting our travel distance error along the path
  private PIDController distanceController = new PIDController(0, 0, 0);

  // Used for correcting our heading error along the path
  private PIDController directionController = new PIDController(0, 0, 0);

  private Notifier pathNotifier = new Notifier(this::moveToNextSegment);
  private Notifier pidNotifier = new Notifier(this::calculateOutputs);

  // The trajectories to follow for each side
  private Trajectory leftTrajectory;
  private Trajectory rightTrajectory;

  private int currentSegment;
  private boolean isFinished;

  /**
   * This will import the path files based on the name of the path provided
   * 
   * @param pathName the name of the path to run
   */
  public PathFollower(String pathName) {
    requires(getDrivetrain());
    importPath(pathName);
  }

  @Override
  protected void initialize() {
    //Make sure we're starting at the beginning of the path
    distanceController.reset();
    directionController.reset();
    currentSegment = 0;
    isFinished = false;

    // Start running the path
    pathNotifier.startPeriodic(leftTrajectory.get(0).dt);
    pidNotifier.startPeriodic(distanceController.getPeriod());

    // If there  was an issue with importing the paths then we should just finish this command instantly
    if (leftTrajectory == null || rightTrajectory == null) {
      HelixEvents.getInstance().addEvent("DRIVETRAIN", "There was an issue importing the path, ending the command instantly.");
      isFinished = true;
    }
  }

  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  @Override
  protected void end() {
    pathNotifier.stop();
    pidNotifier.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }

  private void importPath(String pathName) {
    try {
      // Read the path files from the file system
      leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
      rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
    } catch (IOException e) {
		  e.printStackTrace();
	  }
  }

  private void moveToNextSegment() {
    // Move to the next segment in the path
    currentSegment++;

    // Was that the last segment in our path?
    if (currentSegment >= leftTrajectory.length()) {
      isFinished = true;
    }
  }

  private void calculateOutputs() {
    // If we're finished there are no more segments to read from and we should return
    if (isFinished) {
      return;
    }

    // Get our expected velocities based on the paths
    double leftVelocity = leftTrajectory.get(currentSegment).velocity;
    double rightVelocity = rightTrajectory.get(currentSegment).velocity;

    // Set our expected position to be the setpoint of our distance controller
    // The position will be an average of both the left and right to give us the overall distance
    double expectedPosition = (leftTrajectory.get(currentSegment).position + rightTrajectory.get(currentSegment).position) / 2.0;
    distanceController.setReference(expectedPosition);
    double currentPosition = (getDrivetrain().getLeftPosition() + getDrivetrain().getRightPosition()) / 2.0;

    // Set our expected heading to be the setpoint of our direction controller
    double expectedHeading = leftTrajectory.get(currentSegment).heading;
    distanceController.setReference(expectedHeading);
    double currentHeading = getDrivetrain().getHeading();

    // The final velocity is going to be a combination of our expected velocity corrected by our distance error and our heading error
    // velocity = expected + distanceError +/ headingError
    double correctedLeftVelocity = leftVelocity + distanceController.calculate(currentPosition) + directionController.calculate(currentHeading);
    double correctedRightVelocity = rightVelocity + distanceController.calculate(currentPosition) - directionController.calculate(currentHeading);

    getDrivetrain().setVelocityOutput(correctedLeftVelocity, correctedRightVelocity);
  }
}
