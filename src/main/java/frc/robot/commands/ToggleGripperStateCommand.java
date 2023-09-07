// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class ToggleGripperStateCommand extends CommandBase {
  private final Gripper m_gripperSubsystem;

  /** Creates a new ToggleGripperStateCommand. */
  public ToggleGripperStateCommand(Gripper gripperSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(gripperSubsystem);
    this.m_gripperSubsystem = gripperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_gripperSubsystem.open();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_gripperSubsystem.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
