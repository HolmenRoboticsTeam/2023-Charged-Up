// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
// Everything here is a final because they are constant. They should not change UNLESS we directly change it.
public final class Constants {

  public static final class AutonomousConstants {
    public static final double kMaxVelocity = 2.0;                   //max velocity in 4 m/s
    public static final double kMaxAcceleration = 3.0;               //max acceleration in 3 m/s^2

    public static final HashMap<String, Command> eventMap = new HashMap<>();

    public static final PIDConstants kPIDTranslationAuto = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kPIDRotationAuto = new PIDConstants(0.5, 0.0, 0.0);

    public static final PathConstraints kPathConstraintsAuto = new PathConstraints(kMaxVelocity, kMaxAcceleration);
  }

  public static final class DrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather the
    // reasonably allowed maximums
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2.0 * Math.PI;  // radians per second
    public static final double kMaxAngularSpeedSquared = 2.0 * Math.PI;  // radians per second
    public static final double kSwerveTargetingOffset = 630.0;  // degrees

    // Spark MAX CAN IDs
    public static final int kFrontLeftDriveCANID = 1;
    public static final int kFrontRightDriveCANID = 3;
    public static final int kBackLeftDriveCANID = 5;
    public static final int kBackRightDriveCANID = 7;

    public static final int kFrontLeftSteeringCANID = 2;
    public static final int kFrontRightSteeringCANID = 4;
    public static final int kBackLeftSteeringCANID = 6;
    public static final int kBackRightSteeringCANID = 8;

    // Module Angular Offsets - Relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffsetRadians = -Math.PI / 2.0;
    public static final double kFrontRightChassisAngularOffsetRadians = 0.0;
    public static final double kBackLeftChassisAngularOffsetRadians = Math.PI;
    public static final double kBackRightChassisAngularOffsetRadians = Math.PI / 2.0;

    // Chassis Configuration
    // Distance between the centers of the right and left wheels on the robot
    public static final double kTrackWidthMeters = Units.inchesToMeters(23.0);
    // Distance between the centers of the front and back wheels on the robot
    public static final double kWheelBaseMeters = Units.inchesToMeters(23.0);

    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
      new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)
    };
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);

    public static final double kSteerP = 0.1;
    public static final double kSteerI = 0.01;
    public static final double kSteerD = 0.0001;
    public static final TrapezoidProfile.Constraints kSteerControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularSpeedSquared
    );

    public static final boolean kGyroInverted = false;
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

  public static class OIConstants {
    public static final double kcontrollerDeadband = 0.12;
    public static final int kdriveControllerPort = 0;
  }


}
