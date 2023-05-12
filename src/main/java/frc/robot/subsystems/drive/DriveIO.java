// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double getHeading;
    }
    public default void updateInputs(DriveIOInputs inputs) {}
    public default double getHeading() {return 0.0;}
}
