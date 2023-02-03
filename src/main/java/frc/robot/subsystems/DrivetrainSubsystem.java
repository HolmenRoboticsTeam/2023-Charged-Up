// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Hashtable;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
    this.setSimulatedAngle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
  public Pose2d getPose() {
   return this.m_odometry.getPoseMeters();
  }

  /**
    * Sets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
  public void setPose(Pose2d pose) {
    this.m_odometry.resetPosition(
      Rotation2d.fromDegrees(-this.m_navX.getRotation2d().getDegrees()),
      new SwerveModulePosition[] {
          this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getPosition(),
          this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getPosition(),
          this.m_swerveModules.get(SwerveModule.BACK_LEFT).getPosition(),
          this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getPosition()
      },
      pose
    );
  }

  /**
    * Method to drive the robot using joystick info.
    *
    * @param xSpeed        Speed of the robot in the x direction (forward).
    * @param ySpeed        Speed of the robot in the y direction (sideways).
    * @param rot           Angular rate of the robot.
    * @param fieldRelative Whether the provided x and y speeds are relative to the
    *                      field.
    */
  public void drive(double throttle, double strafe, double rotation, boolean fieldRelative) {
    // Adjust input based on max speed
    throttle *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
    strafe *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
    rotation *= DrivetrainConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, Rotation2d.fromDegrees(-this.m_navX.getRotation2d().getDegrees()))
        : new ChassisSpeeds(throttle, strafe, rotation)
    );

    this.setModuleStates(swerveModuleStates);
  }

  public void drive(double throttle, double strafe, double xRotation, double yRotation, boolean fieldRelative) {
    // Calculate the current angle of the robot
    double currentAngle = Math.abs(this.m_navX.getAngle() % 360);
    if (this.m_navX.getAngle() < 0) currentAngle = 360 - currentAngle;

    // Use SOH CAH (TOA) to determine the desired angle
    // Apply a bias to ensure the front of the robot follows the joystick
    double desiredAngle = Math.atan2(yRotation, xRotation);
    desiredAngle = (Units.radiansToDegrees(desiredAngle) + DrivetrainConstants.kSwerveTargetingOffset) % 360;

    // Calculate the theta difference between current and desired
    // Then calculate the shortest direction to take from current to desired (i.e. -90 CCW is shorter than 270 CW)
    double theta = Math.abs(desiredAngle - currentAngle) % 360;
    double thetaShortened = theta > 180 ? 360 - theta : theta;

    // Calculate corrective action
    double sign = -1;
    if (currentAngle - desiredAngle >= 0 && currentAngle - desiredAngle <= 180) {
      sign = 1;
    } else if (currentAngle - desiredAngle <= -180 && currentAngle - desiredAngle >= -360) {
      sign = 1;
    }
    double correctiveTheta = thetaShortened * sign;  // Figure out which way to turn
    double correctiveTurn = -this.m_smoothSteerController.calculate(correctiveTheta);

    // Send the drive speed to the main drive method
    this.drive(throttle, strafe, correctiveTheta, fieldRelative);
  }

  /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180
    */
  public double getHeadingDegrees() {
    return Math.IEEEremainder(this.m_navX.getAngle(), 360);
  }

  /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180 as Rotation2d
    */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(this.getHeadingDegrees());
  }

  /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
  // public get... 
  public double getTurnRate() {
    return this.m_navX.getRate();
  }

  /**
   * Getting the positions and heading of each Swerve Module
   * @return Meters and Degrees
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getPosition(),
      this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getPosition(),
      this.m_swerveModules.get(SwerveModule.BACK_LEFT).getPosition(),
      this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getPosition()
    };
  }

  /**
   * Getting swerve speed and direction.
   * @return M/S and Degrees
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getState(),
      this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getState(),
      this.m_swerveModules.get(SwerveModule.BACK_LEFT).getState(),
      this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getState()
    };
  }

  /**
    * Sets the swerve speed and direction.
    *
    * @param desiredStates The desired SwerveModule states.
    */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);

    this.m_swerveModules.get(SwerveModule.FRONT_LEFT).setDesiredState(desiredStates[SwerveModule.FRONT_LEFT.getValue()]);
    this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).setDesiredState(desiredStates[SwerveModule.FRONT_RIGHT.getValue()]);
    this.m_swerveModules.get(SwerveModule.BACK_LEFT).setDesiredState(desiredStates[SwerveModule.BACK_LEFT.getValue()]);
    this.m_swerveModules.get(SwerveModule.BACK_RIGHT).setDesiredState(desiredStates[SwerveModule.BACK_RIGHT.getValue()]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.m_navX.reset();
    this.m_simYaw = 0;
  }

  private void setSimulatedAngle(double angle) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    simAngle.set(angle);
  }

}