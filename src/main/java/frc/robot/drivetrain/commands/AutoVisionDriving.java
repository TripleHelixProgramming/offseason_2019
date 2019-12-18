/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import frc.robot.drivetrain.Drivetrain;

public class AutoVisionDriving extends AbstractVisionDriving {

    private double cruisingVelocity;
    private double acceleration;
    private double currentVelocity;
    private Drivetrain drivetrain = Drivetrain.getDrivetrain();

    public AutoVisionDriving(double cruisingVelocity, double acceleration) {
        super();
        this.cruisingVelocity = cruisingVelocity;
        this.acceleration = acceleration *= 0.2;
    }

    @Override
    protected void initialize() {
        super.initialize();
        currentVelocity = (drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2.0;

        if (currentVelocity < 0) {
            acceleration *= -1;
        }
    }

    @Override
    public double getThrottle() {
        // Slow down/speed up to cruise velocity
        double velocityError = currentVelocity - cruisingVelocity;
        if (velocityError == 0) {
            return cruisingVelocity;
        } else if (Math.abs(velocityError) < acceleration) {
            currentVelocity = cruisingVelocity;
            return cruisingVelocity;
        } else if (currentVelocity < cruisingVelocity) {
            currentVelocity += acceleration;
        } else {
            currentVelocity -= acceleration;
        }
        return currentVelocity;
      }
}
