// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrainSubsystem2;

public class DefaultDriveCommand2 extends CommandBase {
  /** Creates a new ControllerCommand. */

  private final XboxController m_controller;
  private final DriveTrainSubsystem2 m_drivetrainSubsystem;

  
  public DefaultDriveCommand2(XboxController controller, DriveTrainSubsystem2 drivetrainSubsystem2) {
    this.m_controller = controller;
    this.m_drivetrainSubsystem = drivetrainSubsystem2;
    this.addRequirements(m_drivetrainSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = MathUtil.applyDeadband(-this.m_controller.getLeftY(), OIConstants.kControllerDeadband);
    double strafe = MathUtil.applyDeadband(-this.m_controller.getLeftX(), OIConstants.kControllerDeadband);
    double rotation =  MathUtil.applyDeadband(this.m_controller.getRightX(), OIConstants.kControllerDeadband);
    this.m_drivetrainSubsystem.drive(throttle, strafe, rotation, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_drivetrainSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
