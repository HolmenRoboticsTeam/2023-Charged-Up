// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutonomousConstants;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load1-CrossAuto-ChargeStation",
    new PathConstraints(AutonomousConstants.kMaxVelocity, AutonomousConstants.kMaxAcceleration));
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
      eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
      eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    // SwerveAutoBuilder autoBuilder1 = new SwerveAutoBuilder(
    // driveSubsystem::getPose, // Pose2d supplier
    // driveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    // driveSubsystem.kinematics, // SwerveDriveKinematics
    // new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    // new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    // driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    // eventMap,
    // true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    // driveSubsystem );// The drive subsystem. Used to properly set the requirements of path following commands
    return null;
  }
// Blue-Load1-CrossAuto-ChargeStation
  public Command blueLoad1CrossAndBalanceAuto() {

    return null;
  }
// Blue-Load3-CrossAuto
  public Command blueLoad3CrossAuto() {

    return null;
  }
// Blue-Load2-CrossAuto
  public Command blueLoad2CrossAuto() {

    return null;
  }
}
