// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathAndEventSubsystem extends SubsystemBase {
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Autonomous Map", new PathConstraints(AutonomousConstants.kMaxVelocity, AutonomousConstants.kMaxAcceleration));
  // ^^^ Underlined because it needs import
  HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new PathAndEventSubsystem. */
  public PathAndEventSubsystem(SwerveAutoBuilder autoBuilder) {
    // String Key, Command value
    eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
    eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

    Command autonomousMap = autoBuilder.fullAuto(pathGroup);



  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
