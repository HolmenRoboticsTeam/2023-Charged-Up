// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double boomLength;
        public Rotation2d boomAngle;
    }
    public default void updateInputs(ArmIOInputs inputs) {}
    public default Rotation2d getAngle() {return null;}
    public default double getBoomLength() {return 0.0;}
    public default void setPivot() {}
    public default void SetBoomLength() {}
    }
    // public default Rotation2d getAngle() {
    //     return Rotation2d.fromDegrees(m_pivotIntEncoder.getPosition());
    // }
