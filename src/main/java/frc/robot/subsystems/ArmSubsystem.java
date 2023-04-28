// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_armExtendSparkMax;
  private final CANSparkMax m_armPivotSparkMax;

  private final RelativeEncoder m_extendAltEncoder;
  private final AbsoluteEncoder m_pivotAbsEncoder;
  private final RelativeEncoder m_pivotIntEncoder;
  private final RelativeEncoder m_extendIntEncoder;

  private final SparkMaxPIDController m_extendPIDController;
  private final SparkMaxPIDController m_pivotPIDController;

  private final double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_pivotConstraints;
  private TrapezoidProfile.State m_pivotGoal;
  private TrapezoidProfile.State m_pivotInitialState;
  private TrapezoidProfile m_profile;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    this.m_armExtendSparkMax = new CANSparkMax(ArmConstants.kExtendCanId, MotorType.kBrushless);
    this.m_armPivotSparkMax = new CANSparkMax(ArmConstants.kPivotCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    this.m_armExtendSparkMax.restoreFactoryDefaults();
    this.m_armPivotSparkMax.restoreFactoryDefaults();
    this.m_armPivotSparkMax.setInverted(true);
    this.m_armExtendSparkMax.setInverted(true);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    this.m_extendAltEncoder = this.m_armExtendSparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    this.m_pivotAbsEncoder = this.m_armPivotSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    this.m_pivotIntEncoder = this.m_armPivotSparkMax.getEncoder();
    this.m_extendIntEncoder = this.m_armExtendSparkMax.getEncoder();
    this.m_extendPIDController = this.m_armExtendSparkMax.getPIDController();
    this.m_pivotPIDController = this.m_armPivotSparkMax.getPIDController();
    // this.m_extendPIDController.setFeedbackDevice(this.m_extendAltEncoder);

    // Apply position conversion factors for the pivot and extend encoders. The native units for
    // position are rotations, but we want meters
    this.m_pivotAbsEncoder.setInverted(true);
    // this.m_extendAltEncoder.setInverted(true);
    // this.m_extendAltEncoder.setPositionConversionFactor(ArmConstants.kExtendEncoderPositionFactor);
    this.m_pivotAbsEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderPositionFactor);
    this.m_pivotAbsEncoder.setVelocityConversionFactor(ArmConstants.kPivotEncoderVelocityFactor);
    this.m_pivotIntEncoder.setPositionConversionFactor(ArmConstants.kInternalPivotEncoderPositionFactor);
    this.m_pivotIntEncoder.setVelocityConversionFactor(ArmConstants.kInternalPivotEncoderVelocityFactor);

    double absPosition = this.m_pivotAbsEncoder.getPosition();
    if (absPosition > 180) {
      absPosition -= 360.0;
    }
    this.m_pivotIntEncoder.setPosition(absPosition);
    // this.m_extendAltEncoder.setPosition(0);

    this.m_pivotConstraints = new TrapezoidProfile.Constraints(ArmConstants.kPivotMaxVel, ArmConstants.kPivotMaxAcc);
    this.m_pivotInitialState = new TrapezoidProfile.State(this.m_pivotIntEncoder.getPosition(), this.m_pivotIntEncoder.getVelocity());
    this.m_pivotGoal = new TrapezoidProfile.State(this.m_pivotIntEncoder.getPosition(), this.m_pivotIntEncoder.getVelocity());
    this.m_profile = new TrapezoidProfile(m_pivotConstraints, m_pivotGoal, m_pivotInitialState);

    // Enable PID wrap around for the pivot motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    this.m_pivotPIDController.setPositionPIDWrappingEnabled(true);
    this.m_pivotPIDController.setPositionPIDWrappingMinInput(ArmConstants.kPivotEncoderPositionPIDMinInput);
    this.m_pivotPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kPivotEncoderPositionPIDMaxInput);

    // Set the PID gains for the extension motor.
    this.m_extendPIDController.setP(ArmConstants.kExtendP);
    this.m_extendPIDController.setI(ArmConstants.kExtendI);
    this.m_extendPIDController.setD(ArmConstants.kExtendD);
    this.m_extendPIDController.setFF(ArmConstants.kExtendFF);
    this.m_extendPIDController.setOutputRange(
      ArmConstants.kExtendMinOutput,
      ArmConstants.kExtendMaxOutput
    );

    // Set the PID gains for the pivot motor.
    this.m_pivotPIDController.setP(ArmConstants.kPivotP);
    this.m_pivotPIDController.setI(ArmConstants.kPivotI);
    this.m_pivotPIDController.setD(ArmConstants.kPivotD);
    this.m_pivotPIDController.setFF(ArmConstants.kPivotFF);
    this.m_pivotPIDController.setOutputRange(
      ArmConstants.kPivotMinOutput,
      ArmConstants.kPivotMaxOutput
    );

    this.m_pivotPIDController.setSmartMotionMaxVelocity(ArmConstants.kPivotMaxVel, 0);
    this.m_pivotPIDController.setSmartMotionMinOutputVelocity(ArmConstants.kPivotMinVel, 0);
    this.m_pivotPIDController.setSmartMotionMaxAccel(ArmConstants.kPivotMaxAcc, 0);
    this.m_pivotPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.kPivotAllowedError, 0);

    this.m_extendPIDController.setSmartMotionMaxVelocity(ArmConstants.kExtendMaxVel, 0);
    this.m_extendPIDController.setSmartMotionMinOutputVelocity(ArmConstants.kExtendMinVel, 0);
    this.m_extendPIDController.setSmartMotionMaxAccel(ArmConstants.kExtendMaxAcc, 0);
    this.m_extendPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.kExtendAllowedError, 0);

    this.m_armExtendSparkMax.setIdleMode(ArmConstants.kExtendMotorIdleMode);
    this.m_armPivotSparkMax.setIdleMode(ArmConstants.kPivotMotorIdleMode);
    this.m_armExtendSparkMax.setSmartCurrentLimit(ArmConstants.kExtendMotorCurrentLimit);
    this.m_armPivotSparkMax.setSmartCurrentLimit(ArmConstants.kPivotMotorCurrentLimit);

    // this.m_extendAltEncoder.setInverted(ArmConstants.kExtendEncoderInverted);
    this.m_armPivotSparkMax.setOpenLoopRampRate(ArmConstants.kPivotRampRateInSeconds);

    this.m_extendIntEncoder.setPosition(0);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    this.m_armExtendSparkMax.burnFlash();
    this.m_armPivotSparkMax.burnFlash();
  }

  /** Get the arm angle from vertical in degrees (Rotation2d).
   *
   * TODO: Need to test; not reliable.
   * @return the arm angle from vertical in degrees (Rotation2d).
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_pivotIntEncoder.getPosition());
  }

  /** Get the length of the arm's boom in meters.
   * TODO: Need to test; not reliable.
   *
   * @return the length of the arm's boom in meters.
   */
  public double getBoomLength() {
    return this.m_extendIntEncoder.getPosition();
  }

  /** Set the arm's boom length and pivot angle.
   *
   * @param desiredState the arm's boom length in meters and pivot angle in degrees.
   */
  public void setDesiredState(SwerveModulePosition desiredState) {
    this.setPivot(desiredState.angle.getDegrees());
    this.setBoomLength(desiredState.distanceMeters);
  }

  public void setPivot(double angleDegrees) {
    this.m_pivotPIDController.setReference(angleDegrees, ControlType.kSmartMotion);
  }

  public void setBoomLength(double rotations) {
    this.m_extendPIDController.setReference(rotations, ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putNumber("Arm Boom Length Meters", this.getBoomLength());
    SmartDashboard.putNumber("Arm Angle Degrees", -this.m_pivotAbsEncoder.getPosition() - 180.0);
    SmartDashboard.putNumber("Arm pivot trapezoid", this.m_pivotInitialState.position);
  }
}
