/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.commands.HelixDrive;
import com.team2363.utilities.RollingAverager;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.Drivetrain.CommandUnits;
import frc.robot.oi.OI;

public class SampleDrive extends HelixDrive {

    private RollingAverager throttle = new RollingAverager(7);

    public SampleDrive() {
        requires(Drivetrain.getDrivetrain());
    }

    @Override
    protected void initialize() {
        super.initialize();
        for (int i = 0; i < 7; i++) {
            throttle.getNewAverage(0);
        }
    }

    @Override
    protected double getThrottle() {
        double newThrottle = OI.getOI().getThrottle();
        if (Math.abs(newThrottle) < 0.05) {
            newThrottle = 0;
        }
        return throttle.getNewAverage(newThrottle);
    }

    @Override
    protected double getTurn() {
        if (Math.abs(OI.getOI().getTurn()) < 0.05) {
            return 0;
        }
        return OI.getOI().getTurn() * 0.5;
    }

    @Override
    protected void useOutputs(double left, double right) {
        Drivetrain.getDrivetrain().setSetpoint(CommandUnits.PERCENT_FULLSPEED, left, right);
    }
}
