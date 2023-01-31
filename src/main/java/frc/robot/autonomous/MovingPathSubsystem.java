// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;

// path is only moving.

public class MovingPathSubsystem extends SubsystemBase {

List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Moving Path1", new PathConstraints(AutonomousConstants.kMaxVelocity, AutonomousConstants.kMaxAcceleration));

  /** Creates a new MovingPathSubsystem. */
  public MovingPathSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
