// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerveModule;
import frc.utils.SwerveUtils;

/** Add your docs here. */
public class DriveIOReal implements DriveIO {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

// The gyro sensor
private final AHRS m_gyro = new AHRS(Port.kUSB1);
private final ScheduledThreadPoolExecutor m_executor = new ScheduledThreadPoolExecutor(1);

//Motion profiling
private final ProfiledPIDController m_smoothSteerController = new ProfiledPIDController(
    DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD, DriveConstants.kHeadingControllerConstraints);

// Slew rate filter variables for controlling lateral acceleration
private double m_currentRotation = 0.0;
private double m_currentTranslationDir = 0.0;
private double m_currentTranslationMag = 0.0;

private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
private double m_prevTime = WPIUtilJNI.now() * 1e-6;

// Odometry class for tracking robot pose
SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(this.getHeading()),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });

    public DriveIOReal() {
        this.m_executor.schedule(this::zeroHeading, (long)1.0, TimeUnit.SECONDS);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(this.getHeading()),
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
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
          // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

          // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
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
          m_currentRotation = m_rotLimiter.calculate(rot);
        } else {
          xSpeedCommanded = xSpeed;
          ySpeedCommanded = ySpeed;
          m_currentRotation = rot;
        }
    
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(this.getHeading()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
      }
    
      public void headingDrive(double throttle, double strafe, double desiredAngle, boolean fieldRelative) {
        double currentAngle = Math.abs(this.m_gyro.getAngle() % 360);
        if (this.m_gyro.getAngle() < 0) currentAngle = 360 - currentAngle;
    
        // Calculate theta difference between current and desired
        desiredAngle = (Units.radiansToDegrees(desiredAngle) + DriveConstants.kSwerveTargetingOffset) % 360;
    
        // Then calculate the shortest direction to take from current to desired
        double theta = Math.abs(desiredAngle - currentAngle) % 360;
        double thetaShortened = theta > 180 ? 360 - theta : theta;
    
        //Calculate Corrective Action
        double sign = -1;
        if (currentAngle - desiredAngle >= 0 && currentAngle - desiredAngle <= 180) {
          sign = 1;
        } else if (currentAngle - desiredAngle <= -180 && currentAngle - desiredAngle >= -360) {
          sign = 1;
        }
        double correctiveTheta = thetaShortened * sign; //Figures out which way to turn
        double correctiveTurn = -this.m_smoothSteerController.calculate(correctiveTheta);
    
        //Send the drive speed to the main drive method
        this.drive(throttle, strafe, correctiveTurn, true, true);
      }
    
      public void headingDrive(double throttle, double strafe, double xRotation, double yRotation, boolean fieldRelative) {
        // Calculate theta difference between current and desired
        double desiredAngle = Math.atan2(yRotation, xRotation);
    
        if (Math.abs(xRotation) < OIConstants.kDriveDeadband && Math.abs(yRotation) < OIConstants.kDriveDeadband) {
          this.drive(throttle, strafe, 0.0, true, true);
          return;
        };
    
        //Send the drive speed to the main drive method
        this.headingDrive(throttle, strafe, desiredAngle, true);
      }
    
      /**
       * Sets the wheels into an X formation to prevent movement.
       */
      public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      }
    
      /**
       * Sets the swerve ModuleStates.
       *
       * @param desiredStates The desired SwerveModule states.
       */
      public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
      }
    
      public void setAutoModuleState(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond);
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
        m_gyro.reset();
      }
    
      /**
       * Returns the heading of the robot.
       *
       * @return the robot's heading in degrees, from -180 to 180
       */
      public double getHeading() {
        return this.m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      }
    
      public double getPitch() {
        return this.m_gyro.getPitch();
      }
    
      public double getRoll() {
        return this.m_gyro.getRoll();
      }
    
      /**
       * Returns the turn rate of the robot.
       *
       * @return The turn rate of the robot, in degrees per second
       */
      public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
      }
}
