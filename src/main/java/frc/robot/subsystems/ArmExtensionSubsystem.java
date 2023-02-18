// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
  private CANSparkMax m_extendSparkMax = new CANSparkMax(0, null);

  private AbsoluteEncoder m_exAbsoluteEncoder;
  /** Creates a new ArmExtensionSubsystem. */
  public ArmExtensionSubsystem() {
    this.m_extendSparkMax = m_extendSparkMax;

  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  this.m_extendSparkMax.restoreFactoryDefaults();

  // Setup Encoders and PID
  this.m_exAbsoluteEncoder = m_extendSparkMax.getAbsoluteEncoder(null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
