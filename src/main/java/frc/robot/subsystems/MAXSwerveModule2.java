// THIS IS THE REVISED CODE NATASHA WROTE!!!

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.SwerveModuleConstants;

public class MAXSwerveModule2 {

  private final int kPOSSlot = 0;
  private final int kVELSlot = 1;
  private final int kSIMSlot = 2;

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_steeringSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_steeringEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_steeringPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Simulations
    private double m_simDriveEncoderPosition;
    private double m_simDriveEncoderVelocity;
    private double m_simAngle;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule2(int drivingCANId, int steeringCANId, double chassisAngularOffset) {
    this.m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    this.m_steeringSparkMax = new CANSparkMax(steeringCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    this.m_drivingSparkMax.restoreFactoryDefaults();
    this.m_steeringSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers
    this.m_drivingEncoder = this.m_drivingSparkMax.getEncoder();
    this.m_steeringEncoder = this.m_steeringSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    this.m_drivingPIDController = this.m_drivingSparkMax.getPIDController();
    this.m_steeringPIDController = this.m_steeringSparkMax.getPIDController();

    this.m_drivingPIDController.setFeedbackDevice(this.m_drivingEncoder);
    this.m_steeringPIDController.setFeedbackDevice(this.m_steeringEncoder);


    // Apply position and velocity conversion factors for the driving encoder. The native units for
    // position and velocity are rotations and RPM, respectively, but we want meters and meters per second
    // to use with WPILib's swerve APIs.
    this.m_drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor);
    this.m_drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the steering encoder. We want these in radians
    // and radians per second to use with WPILib's swerve APIs.
    this.m_steeringEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor);
    this.m_steeringEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor);

    // Invert the turning encoder since the output shaft is inverse of the motor in the module.
    this.m_steeringEncoder.setInverted(SwerveModuleConstants.kSteerEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to
    // get to the setpoint i.e. going from 360 degrees to 10 degrees will go through 0 rather than the
    // other direction which is a longer route.
    this.m_steeringPIDController.setPositionPIDWrappingEnabled(SwerveModuleConstants.kSteerWrapEnabled);
    this.m_steeringPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kSteerEncoderPositionPIDMinInput);
    this.m_steeringPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kSteerEncoderPositionPIDMaxInput);

    // Set the PIDFF gains and output range for the drive motor
    this.m_drivingPIDController.setFF(SwerveModuleConstants.kDriveFF);
    this.m_drivingPIDController.setP(SwerveModuleConstants.kDriveP);
    this.m_drivingPIDController.setI(SwerveModuleConstants.kDriveI);
    this.m_drivingPIDController.setD(SwerveModuleConstants.kDriveD);
    this.m_drivingPIDController.setOutputRange(SwerveModuleConstants.kDriveMinOutputRange, SwerveModuleConstants.kDriveMaxOutputRange);

        // Set the PIDFF gains and output range for the steer motor
        this.m_steeringPIDController.setFF(SwerveModuleConstants.kSteerFF);
        this.m_steeringPIDController.setP(SwerveModuleConstants.kSteerP);
        this.m_steeringPIDController.setI(SwerveModuleConstants.kSteerI);
        this.m_steeringPIDController.setD(SwerveModuleConstants.kSteerD);
        this.m_steeringPIDController.setOutputRange(SwerveModuleConstants.kSteerMinOutputRange, SwerveModuleConstants.kSteerMaxOutputRange);

        // Set the idle mode and current limiting for both drive and steer. Current limiting prevents the motor from tripping
    // the breaker in the PDP resulting in an unusable motor for 15+ seconds
    this.m_drivingSparkMax.setIdleMode(SwerveModuleConstants.kDriveMotorIdleMode);
    this.m_steeringSparkMax.setIdleMode(SwerveModuleConstants.kSteerMotorIdleMode);
    this.m_drivingSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit);
    this.m_steeringSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kSteerMotorCurrentLimit);

    // Burn the configurations. If a SparkMax browns out during operation, it will maintain the above
    // configurations
    this.m_drivingSparkMax.burnFlash();
    this.m_steeringSparkMax.burnFlash();

    // Set chassis offset, the current module state angle position of the steer encoder, and zeros the drive encoder
    // this.m_chassisAngularOffset = chassisAngularOffset;
    // this.m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    // this.m_desiredState.angle = new Rotation2d(this.m_steerEncoder.getPosition());
    // this.m_driveEncoder.setPosition(0);
    this.m_chassisAngularOffset = chassisAngularOffset;
    this.m_desiredState.angle = new Rotation2d(m_steeringEncoder.getPosition());
    this.m_drivingEncoder.setPosition(0);
    
    // Simulation setup
    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(this.m_drivingSparkMax, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(this.m_steeringSparkMax, DCMotor.getNeo550(1));
  }
  }

    /** Returning position of drive encoder in. */
    public double getDrivePosition() {
      return RobotBase.isReal() ? this.m_drivingEncoder.getPosition() : this.m_simDriveEncoderPosition;
    }

  /** Returning speed velocity in M/S */
  public double getDriveVelocity() {
    return RobotBase.isReal() ? this.m_drivingEncoder.getVelocity() : this.m_simDriveEncoderVelocity;
  }

  /** Returning the direction the swerve module is pointing in degrees. */
  public double getHeadingDegrees() {
    return RobotBase.isReal() ? this.m_steeringEncoder.getPosition() : this.m_simAngle;
  }

  /** Returning the direction the swerve module is pointing in degrees. */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(this.getHeadingDegrees());
  }


  // returns current state of motor
  public SwerveModuleState getState() {

    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState( this.m_drivingEncoder.getVelocity(),
        new Rotation2d(m_steeringEncoder.getPosition() - m_chassisAngularOffset));
  }

  // returns current position of motor
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        this.m_drivingEncoder.getPosition(),
        new Rotation2d(this.m_steeringEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.m_drivingEncoder.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(this.m_chassisAngularOffset));

    // Optimize the reference state to avoid rotating the module further than 90 degrees
    // We want the module to take the shortest route possible whether that means its facing forwards or backwards
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
      new Rotation2d(m_steeringEncoder.getPosition()));

    // Set the reference drive and steer states to the motors through the PID controllers to better control
    // harsh changes in speed and direction
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, RobotBase.isReal() ? this.kVELSlot : this.kSIMSlot);
    m_steeringPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition, this.kPOSSlot);

    m_desiredState = optimizedDesiredState;

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
