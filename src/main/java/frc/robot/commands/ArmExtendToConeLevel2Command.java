// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.subsystems.arm.Arm;

public class ArmExtendToConeLevel2Command extends CommandBase {

  private final Arm m_armSubsystem;

  /** Creates a new RetractArmToPositionCommand. */
  public ArmExtendToConeLevel2Command(Arm armSubsystem) {
    this.m_armSubsystem = armSubsystem;
    this.addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_armSubsystem.setBoomLength(ArmPositionConstants.kPlaceConeOnLevel2BoomLength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(Math.abs(ArmPositionConstants.kPlaceConeOnLevel2BoomLength - this.m_armSubsystem.getBoomLength()));
    return Math.abs(ArmPositionConstants.kPlaceConeOnLevel2BoomLength - this.m_armSubsystem.getBoomLength()) < 100.0;
  }
}
