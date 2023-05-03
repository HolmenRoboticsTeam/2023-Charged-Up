// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.compressor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.compressor.CompressorIO.CompressorIOInputs;

public class Compressor extends SubsystemBase {
  private CompressorIO io;
  private CompressorIOInputsAutoLogged inputs = new CompressorIOInputsAutoLogged();
  /** Creates a new Gripper. */
  public Compressor(CompressorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  io.updateInputs(inputs);
  Logger.getInstance().processInputs("Compressor", inputs);
  }

  public void disable() {}
  public void enable() {}
}
