// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MAXSwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveSparkMax;
  private final CANSparkMax m_steerSparkMax;
  /** Creates a new MAXSwerveModule. */
  public MAXSwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
