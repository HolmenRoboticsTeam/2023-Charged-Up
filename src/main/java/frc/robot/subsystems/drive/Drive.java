// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  /** Creates a new Drive. */
  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    
  io.updateOdometry();

  io.updateInputs(inputs);
  Logger.getInstance().processInputs("Drive", inputs);
  }

  public Pose2d getPose() {
    return this.io.getPose();
  }

  public void setPose(Pose2d pose) {
    this.io.setPose(pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    this.io.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
  }

  public void headingDrive(double throttle, double strafe, double desiredAngle, boolean fieldRelative) {
    this.io.headingDrive(throttle, strafe, desiredAngle, fieldRelative);
  }

  public void headingDrive(double throttle, double strafe, double xRotation, double yRotation, boolean fieldRelative) {
    this.io.headingDrive(throttle, strafe, xRotation, yRotation, fieldRelative);
  }

  public void setX() {
    this.io.setX();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    this.io.setModuleStates(desiredStates);
  }

  public void setAutoModuleState(SwerveModuleState[] desiredStates) {
    this.io.setAutoModuleState(desiredStates);
  }

  public void resetEncoders() {
    this.io.resetEncoders();
  }

  public void zeroHeading() {
    this.io.zeroHeading();
  }

  public double getHeading() {
    return this.io.getHeading();
  }

  public double getPitch() {
    return this.io.getPitch();
  }

  public double getRoll() {
    return this.io.getRoll();
  }

  public double getTurnRate() {
    return this.io.getTurnRate();
  }
}
