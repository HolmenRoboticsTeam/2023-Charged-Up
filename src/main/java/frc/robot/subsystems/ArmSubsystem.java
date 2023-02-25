// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_armExtendSparkMax;
  private final CANSparkMax m_armPivotSparkMax;

  private final AbsoluteEncoder m_armExtendEncoder;
  private final AbsoluteEncoder m_armPivotEncoder;

  private final SparkMaxPIDController m_armExtendPIDController;
  private final SparkMaxPIDController m_armPivotPIDController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    this.m_armExtendSparkMax = new CANSparkMax(ArmConstants.kExtendCanId, MotorType.kBrushless);
    this.m_armPivotSparkMax = new CANSparkMax(ArmConstants.kPivotCanId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    this.m_armExtendSparkMax.restoreFactoryDefaults();
    this.m_armPivotSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    this.m_armExtendEncoder = this.m_armExtendSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    this.m_armPivotEncoder = this.m_armPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    this.m_armExtendPIDController = this.m_armExtendSparkMax.getPIDController();
    this.m_armPivotPIDController = this.m_armPivotSparkMax.getPIDController();
    this.m_armExtendPIDController.setFeedbackDevice(this.m_armExtendEncoder);
    this.m_armPivotPIDController.setFeedbackDevice(this.m_armPivotEncoder);

    // Apply position conversion factors for the pivot and extend encoders. The native units for
    // position are rotations, but we want meters
    this.m_armExtendEncoder.setPositionConversionFactor(ArmConstants.kExtendEncoderPositionFactor);
    this.m_armPivotEncoder.setPositionConversionFactor(ArmConstants.kPivotEncoderPositionFactor);

    // Enable PID wrap around for the pivot motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    this.m_armPivotPIDController.setPositionPIDWrappingEnabled(true);
    this.m_armPivotPIDController.setPositionPIDWrappingMinInput(ArmConstants.kPivotEncoderPositionPIDMinInput);
    this.m_armPivotPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kPivotEncoderPositionPIDMaxInput);

    // Set the PID gains for the extension motor.
    this.m_armExtendPIDController.setP(ArmConstants.kExtendP);
    this.m_armExtendPIDController.setI(ArmConstants.kExtendI);
    this.m_armExtendPIDController.setD(ArmConstants.kExtendD);
    this.m_armExtendPIDController.setFF(ArmConstants.kExtendFF);
    this.m_armExtendPIDController.setOutputRange(
      ArmConstants.kExtendMinOutput,
      ArmConstants.kExtendMaxOutput
    );

    // Set the PID gains for the pivot motor.
    this.m_armPivotPIDController.setP(ArmConstants.kPivotP);
    this.m_armPivotPIDController.setI(ArmConstants.kPivotI);
    this.m_armPivotPIDController.setD(ArmConstants.kPivotD);
    this.m_armPivotPIDController.setFF(ArmConstants.kPivotFF);
    this.m_armPivotPIDController.setOutputRange(
      ArmConstants.kPivotMinOutput,
      ArmConstants.kPivotMaxOutput
    );

    this.m_armExtendSparkMax.setIdleMode(ArmConstants.kExtendMotorIdleMode);
    this.m_armPivotSparkMax.setIdleMode(ArmConstants.kPivotMotorIdleMode);
    this.m_armExtendSparkMax.setSmartCurrentLimit(ArmConstants.kExtendMotorCurrentLimit);
    this.m_armPivotSparkMax.setSmartCurrentLimit(ArmConstants.kPivotMotorCurrentLimit);

    // Set the pivot limits
    this.m_armPivotSparkMax.setSoftLimit(SoftLimitDirection.kForward, (float)ArmConstants.kPivotForwardLimit.getRadians());
    this.m_armPivotSparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmConstants.kPivotReverseLimit.getRadians());

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    this.m_armExtendSparkMax.burnFlash();
    this.m_armPivotSparkMax.burnFlash();
  }

  /** Get the arm angle from vertical in radians (Rotation2d).
   *
   * TODO: Need to test; not reliable.
   * @return the arm angle from vertical in radians (Rotation2d).
   */
  public Rotation2d getAngle() {
    return new Rotation2d(this.m_armPivotEncoder.getPosition());
  }

  /** Get the length of the arm's boom in meters.
   * TODO: Need to test; not reliable.
   *
   * @return the length of the arm's boom in meters.
   */
  public double getBoomLength() {
    return this.m_armExtendEncoder.getPosition();
  }

  /** Set the arm's boom length and pivot angle.
   *
   * @param desiredState the arm's boom length in meters and pivot angle in radians.
   */
  public void setDesiredState(SwerveModulePosition desiredState) {
    this.m_armExtendPIDController.setReference(desiredState.distanceMeters, CANSparkMax.ControlType.kPosition);
    this.m_armPivotPIDController.setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double sineRatioOfRobotArm = Math.sin(this.getAngle().getRadians());
    float maxBoomArmLength = (float)(ArmConstants.kMaxHorizontalDistanceOffRobot / sineRatioOfRobotArm);
    float minBoomArmLength = (float)(ArmConstants.kMinHorizontalDistanceOffRobot);
    this.m_armExtendSparkMax.setSoftLimit(SoftLimitDirection.kForward, maxBoomArmLength);
    this.m_armExtendSparkMax.setSoftLimit(SoftLimitDirection.kReverse, minBoomArmLength);

    // Update telemetry
    SmartDashboard.putNumber("Arm Boom Length Meters", this.getBoomLength());
    SmartDashboard.putNumber("Arm Angle Degrees", this.getAngle().getDegrees());
  }
}
