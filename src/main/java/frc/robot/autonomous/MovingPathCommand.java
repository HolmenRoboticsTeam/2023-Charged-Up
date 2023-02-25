// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class MovingPathCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  /** Creates a new MovingPathCommand. */
  public MovingPathCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command buildSwerveAutoBuilder(List<PathPlannerTrajectory> pathGroup, HashMap<String, Command> eventMap) {

    //Creates the autobuilder. Only needs to be done once.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this.m_driveSubsystem::getPose,             // Pose2d supplier
      this.m_driveSubsystem::resetOdometry,             // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics,            // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
      AutonomousConstants.kPIDTranslationAuto,         // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutonomousConstants.kPIDRotationAuto,            // PID constants to correct for rotation error (used to create the rotation controller)
      this.m_driveSubsystem::setModuleStates,     // Module states consumer used to output to the drive subsystem
      eventMap,
      true,                          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this.m_driveSubsystem);                     // The drive subsystem. Used to properly set the requirements of path following commands

      return autoBuilder.fullAuto(pathGroup);
  }

  public Command getAutonomousCommand() {

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Load 1, cross, and balance", blueLoad1CrossAndBalanceAuto());
    m_chooser.addOption("Load 3, cross", blueLoad3CrossAuto());
    m_chooser.addOption("Load 2, cross", blueLoad2CrossAuto());
    SmartDashboard.putData(m_chooser);

    return m_chooser.getSelected();
  }

// Blue-Load1-CrossAuto-ChargeStation
public Command blueLoad1CrossAndBalanceAuto() {
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load1-CrossAuto-ChargeStation", AutonomousConstants.kPathConstraintsAuto);
  HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new PathAndEventSubsystem. */
  eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
  eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

  return this.buildSwerveAutoBuilder(pathGroup, eventMap);
}

// Blue-Load3-CrossAuto
public Command blueLoad3CrossAuto() {
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load3-CrossAuto", AutonomousConstants.kPathConstraintsAuto);
  HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new PathAndEventSubsystem. */
  eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
  eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    return this.buildSwerveAutoBuilder(pathGroup, eventMap);
  }

public Command blueLoad2CrossAuto() {
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load2-CrossAuto", AutonomousConstants.kPathConstraintsAuto);
  HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new PathAndEventSubsystem. */
  eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
  eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    return this.buildSwerveAutoBuilder(pathGroup, eventMap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
