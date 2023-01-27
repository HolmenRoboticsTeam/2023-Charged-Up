// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathAndEventSubsystem extends SubsystemBase {
  ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Autonomous Map", new PathConstraints(AutonomousConstants.kMaxVelocity, AutonomousConstants.kMaxAcceleration));
  // ^^^ Underlined because it needs import
  HashMap<String, Command> eventMap = new HashMap<>();
  
  /** Creates a new PathAndEventSubsystem. */
  public PathAndEventSubsystem() {
    eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
    eventMap.put("Grab", new IntakeDown()); // will not be intake
  
    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    
    Command autonomousMap = autoBuilder.autonomousMap(pathGroup);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
