// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// path is only moving. 

public class MovingPathSubsystem extends SubsystemBase {
  ArrayList<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("", new PathConstraints(4, 3)); // change constants
  /** Creates a new MovingPathSubsystem. */
  public MovingPathSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
