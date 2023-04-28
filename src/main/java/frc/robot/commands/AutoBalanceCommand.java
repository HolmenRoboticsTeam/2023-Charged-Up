// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final double kOffBalanceAngleThresholdDegrees;
  private final double kOnBalanceAngleThresholdDegrees;
  private boolean m_autoBalanceXMode;
  private boolean m_autoBalanceYMode;


  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_autoBalanceXMode = false;
    this.m_autoBalanceYMode = false;
    this.kOffBalanceAngleThresholdDegrees = 10;
    this.kOnBalanceAngleThresholdDegrees = 5;

    this.addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees    = this.m_driveSubsystem.getPitch();
    double rollAngleDegrees     = this.m_driveSubsystem.getRoll();
    double xAxisRate            = 0;
    double yAxisRate            = 0;

    if (!this.m_autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) >= Math.abs(this.kOffBalanceAngleThresholdDegrees))) {
      this.m_autoBalanceXMode = true;
    } else if (this.m_autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) <= Math.abs(this.kOnBalanceAngleThresholdDegrees))) {
      this.m_autoBalanceXMode = false;
    }
    if (!this.m_autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) >= Math.abs(this.kOffBalanceAngleThresholdDegrees))) {
      this.m_autoBalanceYMode = true;
    } else if (this.m_autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) <= Math.abs(this.kOnBalanceAngleThresholdDegrees))) {
      this.m_autoBalanceYMode = false;
    }

    // Control drive system automatically,
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle

    if (m_autoBalanceXMode) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -0.35;
    }
    if (m_autoBalanceYMode) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * -0.35;
    }

    // System.out.println();
    this.m_driveSubsystem.headingDrive(xAxisRate, yAxisRate, 0.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
