// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command buildSwerveAutoBuilder(List<PathPlannerTrajectory> pathGroup, HashMap<String, Command> eventMap) {

    //Creates the autobuilder. Only needs to be done once.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      DrivetrainSubsystem::getPose,             // Pose2d supplier
      DrivetrainSubsystem::resetPose,           // Pose2d consumer, used to reset odometry at the beginning of auto
      SwerveModuleConstants.kinematicsAuto,           // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
      AutonomousConstants.kPIDTranslationAuto,  // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutonomousConstants.kPIDRotationAuto,     // PID constants to correct for rotation error (used to create the rotation controller)
      DrivetrainSubsystem::setModuleStates,     // Module states consumer used to output to the drive subsystem
      eventMap,
      true,                              // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      DrivetrainSubsystem);                     // The drive subsystem. Used to properly set the requirements of path following commands

      return autoBuilder.fullAuto(pathGroup);
  }

  public Command getAutonomousCommand() {

    return null;
  }
// Blue-Load1-CrossAuto-ChargeStation
  public Command blueLoad1CrossAndBalanceAuto() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load1-CrossAuto-ChargeStation", AutonomousConstants.kPathConstraintsAuto);
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
      eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
      eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    return null;
  }
// Blue-Load3-CrossAuto
  public Command blueLoad3CrossAuto() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load3-CrossAuto", AutonomousConstants.kPathConstraintsAuto);
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
      eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
      eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    return null;
  }
// Blue-Load2-CrossAuto
  public Command blueLoad2CrossAuto() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load2-CrossAuto", AutonomousConstants.kPathConstraintsAuto);
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
      eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
      eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    return null;
  }
}
