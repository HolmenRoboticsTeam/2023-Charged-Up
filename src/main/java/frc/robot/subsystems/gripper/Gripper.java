// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private GripperIO io;
  private GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  /** Creates a new Gripper. */
  public Gripper(GripperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gripper", inputs);
  }

  public void open() {
    this.io.open();
  }
  public void close() {
    this.io.close();
  }
}
