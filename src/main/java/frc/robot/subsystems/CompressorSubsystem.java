// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {

  private Compressor m_compressor;

  /** Creates a new CompressorSubsystem. */
  public CompressorSubsystem() {
    this.m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  }

  /** Disable the compressor. */
  public void disable() {
    this.m_compressor.disable();
  }

  /** Enable the compressor.

  It will automatically turn off once system pressure has been reached (~120 psi).
   */
  public void enable() {
    this.m_compressor.enableDigital();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor Enable State", this.m_compressor.isEnabled());
  }
}
