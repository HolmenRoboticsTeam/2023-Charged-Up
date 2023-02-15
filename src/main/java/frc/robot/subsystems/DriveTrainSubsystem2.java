// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.util.Hashtable;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;

public class DriveTrainSubsystem2 extends SubsystemBase {

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

  private final MAXSwerveModule2 m_frontLeft = new MAXSwerveModule2(
      DrivetrainConstants.kFrontLeftDriveCANID,
      DrivetrainConstants.kFrontLeftSteeringCANID,
      DrivetrainConstants.kFrontLeftChassisAngularOffsetRadians);

  private final MAXSwerveModule2 m_frontRight = new MAXSwerveModule2(
      DrivetrainConstants.kFrontRightDriveCANID,
      DrivetrainConstants.kFrontRightSteeringCANID,
      DrivetrainConstants.kFrontRightChassisAngularOffsetRadians);

  private final MAXSwerveModule2 m_rearLeft = new MAXSwerveModule2(
      DrivetrainConstants.kBackLeftDriveCANID,
      DrivetrainConstants.kBackLeftSteeringCANID,
      DrivetrainConstants.kBackLeftChassisAngularOffsetRadians);

  private final MAXSwerveModule2 m_rearRight = new MAXSwerveModule2(
      DrivetrainConstants.kBackRightDriveCANID,
      DrivetrainConstants.kBackRightSteeringCANID,
      DrivetrainConstants.kBackRightChassisAngularOffsetRadians);

  private final ScheduledThreadPoolExecutor executer = new ScheduledThreadPoolExecutor(1);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(Port.kMXP);

  // MAXSwerveModules
  private Hashtable<SwerveModule, MAXSwerveModule2> m_swerveModules;

  // Odometry
  private final SwerveDriveOdometry m_odometry;

  // Motion profiling
  private final ProfiledPIDController m_smoothSteerController;

  // Simulation
  private double m_simYaw;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.kMagnitudeSlewRate); // need to add constants
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(
    DrivetrainConstants.kDriveKinematics,
    Rotation2d.fromDegrees(m_gyro.getAngle()),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });


    public DriveTrainSubsystem2() { // duplicate method error
    // Swerve Modules
    this.m_swerveModules = new Hashtable<SwerveModule, MAXSwerveModule2>();
    this.m_swerveModules.put(
      SwerveModule.FRONT_LEFT,
      new MAXSwerveModule2(
        DrivetrainConstants.kFrontLeftDriveCANID,
        DrivetrainConstants.kFrontLeftSteeringCANID,
        0  // DrivetrainConstants.kFrontLeftChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.FRONT_RIGHT,
      new MAXSwerveModule2(
        DrivetrainConstants.kFrontRightDriveCANID,
        DrivetrainConstants.kFrontRightSteeringCANID,
        0 // DrivetrainConstants.kFrontRightChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.BACK_LEFT,
      new MAXSwerveModule2(
        DrivetrainConstants.kBackLeftDriveCANID,
        DrivetrainConstants.kBackLeftSteeringCANID,
        0 // DrivetrainConstants.kBackLeftChassisAngularOffsetRadians
      )
    );

    this.m_swerveModules.put(
      SwerveModule.BACK_RIGHT,
      new MAXSwerveModule2(
        DrivetrainConstants.kBackRightDriveCANID,
        DrivetrainConstants.kBackRightSteeringCANID,
        0 // DrivetrainConstants.kBackRightChassisAngularOffsetRadians
      )
    );

    // Motion profiling
    this.m_smoothSteerController = new ProfiledPIDController(
      DrivetrainConstants.kSteerP,
      DrivetrainConstants.kSteerI,
      DrivetrainConstants.kSteerD,
      DrivetrainConstants.kSteerControllerConstraints
    );

    // Odometry
    this.m_odometry = new SwerveDriveOdometry(
        DrivetrainConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-this.m_gyro.getRotation2d().getDegrees()),
        new SwerveModulePosition[] {
            this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getPosition(),
            this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getPosition(),
            this.m_swerveModules.get(SwerveModule.BACK_LEFT).getPosition(),
            this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getPosition()
        },
        new Pose2d(1.80, 4.88, this.getHeading())
    );

    // Simulation & reset the gyro
    this.setSimulatedAngle(0);

    executer.schedule(this.m_gyro::reset, (long)1.0, TimeUnit.SECONDS);
  }

  @Override
  public void periodic() {
  // Update the odometry in the periodic block
  m_odometry.update(
    Rotation2d.fromDegrees(m_gyro.getAngle()),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });
    System.out.println(m_gyro.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = DrivetrainConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
    this.m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
    this.setSimulatedAngle(-Units.radiansToDegrees(this.m_simYaw));
  }


    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void setPose(Pose2d pose) {
    this.m_odometry.resetPosition(
      Rotation2d.fromDegrees(-this.m_gyro.getRotation2d().getDegrees()),
      new SwerveModulePosition[] {
        this.m_swerveModules.get(SwerveModule.FRONT_LEFT).getPosition(),
        this.m_swerveModules.get(SwerveModule.FRONT_RIGHT).getPosition(),
        this.m_swerveModules.get(SwerveModule.BACK_LEFT).getPosition(),
        this.m_swerveModules.get(SwerveModule.BACK_RIGHT).getPosition()
    },
      pose
    );
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

    /**
   * Method to drive the robot using joystick info.
   *
   * @param throttle        Speed of the robot in the x direction (forward).
   * @param strafe        Speed of the robot in the y direction (sideways).
   * @param rotation           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double throttle, double strafe, double rotation, boolean fieldRelative, boolean rateLimit) {
    throttle *= 0.25;
    strafe *= 0.25;
    rotation *= 0.25;

    double xSpeedCommanded;
    double ySpeedCommanded;

    // everything below is for rate
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(strafe, throttle);
      double inputTranslationMag = Math.sqrt(Math.pow(throttle, 2) + Math.pow(strafe, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DrivetrainConstants.kDirectionSlewRate / m_currentTranslationMag); // Constant issues.
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rotation);


    } else {
      xSpeedCommanded = throttle;
      ySpeedCommanded = strafe;
      m_currentRotation = rotation;
    }

    // everything above is rate limit stuff from REV

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DrivetrainConstants.kMaxAngularSpeed;

    var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));


    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double throttle, double strafe, double xRotation, double yRotation, boolean fieldRelative) {

    // Calculate the current angle of the robot
    double currentAngle = Math.abs(this.m_gyro.getAngle() % 360);
    if (this.m_gyro.getAngle() < 0) currentAngle = 360 - currentAngle;

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
    if (Math.abs(xRotation) < OIConstants.kControllerDeadband && Math.abs(yRotation) < OIConstants.kControllerDeadband) correctiveTurn = 0;

    // Send the drive speed to the main drive method
    this.drive(throttle, strafe, xRotation, correctiveTurn, fieldRelative);
    System.out.println(throttle);
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(this.m_gyro.getAngle(), 360);
  }

  /**
  * Returns the desired swerve module.
  * @param desiredModule Which corner module to get.
  * @return The module at the corner specified.
  */
  public MAXSwerveModule2 getModule(SwerveModule desiredModule) {
    return this.m_swerveModules.get(desiredModule);
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
   * Sets the wheels into an X formation to prevent movement. Stolen from Rev's code.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
  public double getTurnRate() {
    return this.m_gyro.getRate() * (DrivetrainConstants.kGyroInverted ? - 1.0 : 1.0);
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
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.m_gyro.reset();
    this.m_simYaw = 0;
  }


  private void setSimulatedAngle(double angle) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    simAngle.set(angle);
  }
}