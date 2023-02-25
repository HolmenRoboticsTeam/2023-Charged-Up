// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {

   // Initialize compressor.
  private final DoubleSolenoid m_valve;

  /** Creates a new GripperSubystem. */
  public GripperSubsystem() {
    this.m_valve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GripperConstants.kForwardChannel, GripperConstants.kReverseChannel);
  }

  public void open() {
    this.m_valve.set(Value.kForward);

  }
  public void close() {
    this.m_valve.set(Value.kReverse);
  }

  @Override
  public void periodic() {
  }
    // This method will be called once per scheduler run
}
