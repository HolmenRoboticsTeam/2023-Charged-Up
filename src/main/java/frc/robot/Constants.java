// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
// Everything here is a final because they are constant. They should not change UNLESS we directly change it.
public final class Constants {

  public static final class AutonomousConstants {
    public static final int kMaxVelocity = 0;                   //max velocity of 4 m/s
    public static final int kMaxAcceleration = 0;               //a max acceleration of 3 m/s^2

    public static final HashMap<String, Command> eventMap = new HashMap<>();
  }

  public static final class SwerveModuleConstants {
    public static final boolean kSteerEncoderInverted = true;

    // Calculation required for driving motor conversion factors and feed forward
    public static final double kPinionTeeth = 13.0;  // Adjust depending on kit chosen
    public static final double kNEOFreeSpeedRPM = 5676 / 60;
    public static final double kDriveMotorReduction = 990.0 / (kPinionTeeth * 15.0);
    public static final double kWheelDiameterMeters =  Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveTrainFreeSpeedMetersPerSecond = (kNEOFreeSpeedRPM * kWheelCircumferenceMeters) / kDriveMotorReduction;  // Calculated motor free speed with module attached

    public static final double kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction;  // Meters
    public static final double kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0;  // Meters Per Second

    public static final double kSteerEncoderPositionFactor = 2.0 * Math.PI;  // Radians
    public static final double kSteerEncoderVelocityFactor = kSteerEncoderPositionFactor / 60.0;  // Radians Per Second

    public static final double kSteerEncoderPositionPIDMinInput = 0;  // Radians
    public static final double kSteerEncoderPositionPIDMaxInput = kSteerEncoderPositionFactor;  // Radians

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 1.0 / kDriveTrainFreeSpeedMetersPerSecond;  // Feed-forward
    public static final double kDriveMinOutputRange = -1.0;
    public static final double kDriveMaxOutputRange = 1.0;

    public static final double kSteerP = 0.04;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;
    public static final double kSteerFF = 0.0;  // Feed-forward
    public static final double kSteerMinOutputRange = -1.0;
    public static final double kSteerMaxOutputRange = 1.0;
    public static final boolean kSteerWrapEnabled = true;

    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kSteerMotorIdleMode = IdleMode.kBrake;

    public static final int kDriveMotorCurrentLimit = 50;  // Amps
    public static final int kSteerMotorCurrentLimit = 20;  // Amps
  }


}
