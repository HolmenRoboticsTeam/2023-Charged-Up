// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Hashtable;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  /** Enums representing each corner swerve module. */
  public static enum SwerveModule {
    FRONT_LEFT(0), FRONT_RIGHT(1), BACK_LEFT(2), BACK_RIGHT(3);

    private final int m_value;

    private SwerveModule(int value) {
      this.m_value = value;
    }

    /** Getting the integer representative number of the Enum.
    *
    * @return Integer
    */
    public int getValue() {
      return this.m_value;
    }

    /**
     * Get the human-readable enum for a specific swerve module.
     * @param value The int representation of the swerve module.
     * @return Enum
     */
    public static SwerveModule toEnum(int value) {
      if (value == SwerveModule.FRONT_LEFT.getValue()) {
        return SwerveModule.FRONT_LEFT;
      } else if (value == SwerveModule.FRONT_RIGHT.getValue()) {
        return SwerveModule.FRONT_RIGHT;
      } else if (value == SwerveModule.BACK_LEFT.getValue()) {
        return SwerveModule.BACK_LEFT;
      } else {
        return SwerveModule.BACK_RIGHT;
      }
    }
  }

  // MAXSwerveModules
  private Hashtable<SwerveModule, MAXSwerveModule> m_swerveModules;

  // The gyro sensor
  private final AHRS m_navX;
  // Odometry
  private final SwerveDriveOdometry m_odometry;

  // Motion profiling
  private final ProfiledPIDController m_smoothSteerController;

  // Simulation
  private double m_simYaw;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // Swerve Modules
    this.m_swerveModules = new Hashtable<SwerveModule, MAXSwerveModule>();
    this.m_swerveModules.put(
      SwerveModule.FRONT_LEFT,
      new MAXSwerveModule(
        DrivetrainConstants.kFrontLeftDriveCANID,
        DrivetrainConstants.kFrontLeftSteeringCANID,
        0  // DrivetrainConstants.kFrontLeftChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.FRONT_RIGHT,
      new MAXSwerveModule(
        DrivetrainConstants.kFrontRightDriveCANID,
        DrivetrainConstants.kFrontRightSteeringCANID,
        0 // DrivetrainConstants.kFrontRightChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.BACK_LEFT,
      new MAXSwerveModule(
        DrivetrainConstants.kBackLeftDriveCANID,
        DrivetrainConstants.kBackLeftSteeringCANID,
        0 // DrivetrainConstants.kBackLeftChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.BACK_RIGHT,
      new MAXSwerveModule(
        DrivetrainConstants.kBackRightDriveCANID,
        DrivetrainConstants.kBackRightSteeringCANID,
        0 // DrivetrainConstants.kBackRightChassisAngularOffsetRadians
      )
    );

    // Gyro
    // SPI.Port.kMXP
    this.m_navX = new AHRS(SPI.Port.kMXP);

    // Odometry
    this.m_odometry = new SwerveDriveOdometry(
        DrivetrainConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-this.m_navX.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getPosition(),
            this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getPosition(),
            this.m_swerveModules.get(SwerveModule.BACK_LEFT).getPosition(),
            this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getPosition()
        },
        new Pose2d(1.80, 4.88, Rotation2d.fromDegrees(0)) // Change to getHeading
    );

    // Motion profiling
    this.m_smoothSteerController = new ProfiledPIDController(
      DrivetrainConstants.kSteerP,
      DrivetrainConstants.kSteerI,
      DrivetrainConstants.kSteerD,
      DrivetrainConstants.kSteerControllerConstraints
    );

    // Simulation & reset the gyro
    // self.setSimulatedAngle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
