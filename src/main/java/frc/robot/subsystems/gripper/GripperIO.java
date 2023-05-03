// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GripperIO {

    @AutoLog
    public static class GripperIOInputs {
        public boolean isGripperOpen = true;
    }
    public default void updateInputs(GripperIOInputs inputs) {}
    public default void open() {}
    public default void close() {}

    
}
