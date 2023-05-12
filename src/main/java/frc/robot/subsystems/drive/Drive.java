// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private DriveIO io;
  // private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  /** Creates a new Drive. */
  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {


  // io.updateInputs(inputs);
  // Logger.getInstance().processInputs(key: "Drive", inputs);
  }

  public void getHeading() {
    this.io.getHeading();
  }
}
