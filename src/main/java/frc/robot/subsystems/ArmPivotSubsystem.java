// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {

  private CANSparkMax m_pivotSparkMax = new CANSparkMax(0, null);


  private final AbsoluteEncoder m_pivAbsoluteEncoder;


  /** Creates a new ArmControlSubsystem. */
  public ArmPivotSubsystem(CANSparkMax m_pivotSparkMax) {
    this.m_pivotSparkMax = m_pivotSparkMax;

  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
    this.m_pivotSparkMax.restoreFactoryDefaults();

  // Setup Encoders and PID
    this.m_pivAbsoluteEncoder = m_pivotSparkMax.getAbsoluteEncoder(null);




  }








  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
