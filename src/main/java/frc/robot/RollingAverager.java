/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class RollingAverager {

    private List<Double> values;

    public RollingAverager(int queueSize) {
        values = new ArrayList<>();
        for (int i = 0; i < queueSize; i++) {
            values.add(0.0);
        }
    }

    public double getNewAverage(double newValue) {
        values.add(newValue);
        values.remove(0);

        return values.stream().mapToDouble(d -> d).average().getAsDouble();
    }
}
