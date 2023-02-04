// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * The smallest and lightest swerve module designed for FRC Teams.
 *
 * MAXSwerve aims to reduce the complexity traditionally associated with swerve
 * drivetrains. The module
 * features a 3in wheel to ensure that your wheelbase is as wide as possible
 * within your robot footprint.
 * Designed with robustness and serviceability in mind, the module features an
 * all-metal construction with
 * steel drive gears. The rotation (azimuth) is gear-driven from our NEO 550
 * Brushless Motor through an
 * UltraPlanetary Gearbox. The included Through Bore Encoder provides 10-bit
 * angular resolution directly on
 * the turning axis. While the azimuth must be driven with a NEO 550 and SPARK
 * MAX Motor Controller to take
 * advantage of new software features, teams can use either a NEO Brushless
 * Motor or Falcon500 for the main
 * drive motor. These modules work seamlessly with the MAXTube structural
 * pieces, however they are designed to
 * work with any 2x1in aluminum tubing.
 *
 * The module can be purchased here: https://www.revrobotics.com/rev-21-3005/
 */
public class MAXSwerveModule {
  // SparkMAX internal PID loops.
  private final int kPOSSlot = 0;
  private final int kVELSlot = 1;
  private final int kSIMSlot = 2;

  // Motors
  private final CANSparkMax m_driveSparkMax;
  private final CANSparkMax m_steerSparkMax;

  // Encoders
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_steerEncoder;

  // PID Controller
  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_steerPIDController;

  // States
  private double m_chassisAngularOffset = 0.0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Simulations
  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;
  private double m_simAngle;

  /** Constructs a MAXSwerveModule.
    *
    * Configures the driving and steering motors, encoders, and PID controllers. This configuration is
    * specific to the REV MAXSwerve Module built with NEOs, SparkMAXs, and a Through-Bore Encoder connected
    * to the steering SparkMAX.
    */
  public MAXSwerveModule(int driveCANID, int steerCANID, double chassisAngularOffset) {
    this.m_driveSparkMax = new CANSparkMax(driveCANID, MotorType.kBrushless);
    this.m_steerSparkMax = new CANSparkMax(steerCANID, MotorType.kBrushless);

    // Factory reset to a known state. Necessary in the event a SparkMAX is swapped out.
    this.m_driveSparkMax.restoreFactoryDefaults();
    this.m_steerSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers
    this.m_driveEncoder = this.m_driveSparkMax.getEncoder();
    this.m_steerEncoder = this.m_steerSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    this.m_drivePIDController = this.m_driveSparkMax.getPIDController();
    this.m_steerPIDController = this.m_steerSparkMax.getPIDController();

    this.m_drivePIDController.setFeedbackDevice(this.m_driveEncoder);
    this.m_steerPIDController.setFeedbackDevice(this.m_steerEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The native units for
    // position and velocity are rotations and RPM, respectively, but we want meters and meters per second
    // to use with WPILib's swerve APIs.
    this.m_driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor);
    this.m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the steering encoder. We want these in radians
    // and radians per second to use with WPILib's swerve APIs.
    this.m_steerEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor);
    this.m_steerEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor);
  
    // Invert the turning encoder since the output shaft is inverse of the motor in the module.
    this.m_steerEncoder.setInverted(SwerveModuleConstants.kSteerEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to
    // get to the setpoint i.e. going from 360 degrees to 10 degrees will go through 0 rather than the
    // other direction which is a longer route.
    this.m_steerPIDController.setPositionPIDWrappingEnabled(SwerveModuleConstants.kSteerWrapEnabled);
    this.m_steerPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kSteerEncoderPositionPIDMaxInput);
    this.m_steerPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kSteerEncoderPositionPIDMinInput);

    // Set the PIDFF gains and output range for the drive motor
    this.m_drivePIDController.setFF(SwerveModuleConstants.kDriveFF);
    this.m_drivePIDController.setP(SwerveModuleConstants.kDriveP);
    this.m_drivePIDController.setI(SwerveModuleConstants.kDriveI);
    this.m_drivePIDController.setD(SwerveModuleConstants.kDriveD);
    this.m_drivePIDController.setOutputRange(SwerveModuleConstants.kDriveMinOutputRange, SwerveModuleConstants.kDriveMaxOutputRange);

    // Set the PIDFF gains and output range for the steer motor
    this.m_steerPIDController.setFF(SwerveModuleConstants.kSteerFF);
    this.m_steerPIDController.setP(SwerveModuleConstants.kSteerP);
    this.m_steerPIDController.setI(SwerveModuleConstants.kSteerI);
    this.m_steerPIDController.setD(SwerveModuleConstants.kSteerD);
    this.m_steerPIDController.setOutputRange(SwerveModuleConstants.kSteerMinOutputRange, SwerveModuleConstants.kSteerMaxOutputRange);

    // Set the idle mode and current limiting for both drive and steer. Current limiting prevents the motor from tripping
    // the breaker in the PDP resulting in an unusable motor for 15+ seconds
    this.m_driveSparkMax.setIdleMode(SwerveModuleConstants.kDriveMotorIdleMode);
    this.m_steerSparkMax.setIdleMode(SwerveModuleConstants.kSteerMotorIdleMode);
    this.m_driveSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit);
    this.m_steerSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kSteerMotorCurrentLimit);

    // Burn the configurations. If a SparkMax browns out during operation, it will maintain the above
    // configurations
    this.m_driveSparkMax.burnFlash();
    this.m_steerSparkMax.burnFlash();

    // Set chassis offset, the current module state angle position of the steer encoder, and zeros the drive encoder
    this.m_chassisAngularOffset = chassisAngularOffset;
    this.m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    this.m_desiredState.angle = new Rotation2d(this.m_steerEncoder.getPosition());
    this.m_driveEncoder.setPosition(0);

    // Simulation setup
    if (RobotBase.isSimulation()) {
        REVPhysicsSim.getInstance().addSparkMax(this.m_driveSparkMax, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(this.m_steerSparkMax, DCMotor.getNeo550(1));
    }
  }

  /** Returning position of drive encoder in. */
  public double getDrivePosition() {
    return RobotBase.isReal() ? this.m_driveEncoder.getPosition() : this.m_simDriveEncoderPosition;
  }

  /** Returning speed velocity in M/S */
  public double getDriveVelocity() {
    return RobotBase.isReal() ? this.m_driveEncoder.getVelocity() : this.m_simDriveEncoderVelocity;
  }

  /** Returning the direction the swerve module is pointing in degrees. */
  public double getHeadingDegrees() {
    return RobotBase.isReal() ? this.m_steerEncoder.getPosition() : this.m_simAngle;
  }

  /** Returning the direction the swerve module is pointing in degrees. */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(this.getHeadingDegrees());
  }

  /**
   * Returning Swerve module velocity and heading.
   * @return M/S and Degrees
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getDriveVelocity(), this.getHeading());
  }

  /**
   * Returning Swerve module position and heading.
   * @return Meters and Degrees
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDrivePosition(), this.getHeading());
  }

  /** Resetting drive encoder. */
  public void resetDriveEncoder() {
    this.m_driveEncoder.setPosition(0);
  }

  /**
    * Sets the swerve module's speed and direction in meters/second and degrees.

    Additionally corrects for offset in relation to the orgin of the chassis, as well as
    while also choosing the shortest turning route.

    * @param desiredState Desired module velocity and heading in M/S and degrees
    */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(this.m_chassisAngularOffset));

    // Optimize the reference state to avoid rotating the module further than 90 degrees
    // We want the module to take the shortest route possible whether that means its facing forwards or backwards
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, this.getHeading()); // this.getState().angle

    // Limit the reference angle to reduce jittering
    // double angleSetpoint =
    //     Math.abs(optimizedDesiredState.speedMetersPerSecond) < (ModuleConstants.kDriveTrainFreeSpeedMetersPerSecond * 0.01)
    //     ? this.m_desiredState.angle.getDegrees()
    //     : optimizedDesiredState.angle.getDegrees();

    // Set the reference drive and steer states to the motors through the PID controllers to better control
    // harsh changes in speed and direction
    this.m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, RobotBase.isReal() ? this.kVELSlot : this.kSIMSlot);
    this.m_steerPIDController.setReference(optimizedDesiredState.angle.getDegrees(), CANSparkMax.ControlType.kPosition, this.kPOSSlot);
    this.m_desiredState = optimizedDesiredState;

    if (RobotBase.isSimulation()) {
      this.updateSimulatedDrivePosition();
      this.m_simAngle = optimizedDesiredState.angle.getDegrees();
    }
  }

  private void updateSimulatedDrivePosition() {
    this.m_simDriveEncoderVelocity = this.m_desiredState.speedMetersPerSecond;
    double distancePer20Ms = this.m_simDriveEncoderVelocity / 50.0;
    this.m_simDriveEncoderPosition += distancePer20Ms;
  }
}
