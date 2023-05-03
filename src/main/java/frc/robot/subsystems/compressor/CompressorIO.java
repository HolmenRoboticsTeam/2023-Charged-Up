// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.compressor;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CompressorIO {
    @AutoLog
    public static class CompressorIOInputs {
        public boolean compressorIsEnabled = true;
        public boolean compressorIsOn = false;
    }
    public default void updateInputs(CompressorIOInputs inputs) {}
    public default void disable() {}
    public default void enable() {}


}
