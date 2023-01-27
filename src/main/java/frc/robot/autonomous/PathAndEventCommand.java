// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathAndEventCommand extends CommandBase {

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods

// copy and pasted 
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             this.resetOdometry(traj.getInitialHolonomicPose());
         }
       }),
       new PPSwerveControllerCommand(
           traj, 
           this::getPose, // Pose supplier
           this.kinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsystem
       )
   );
}

  /** Creates a new PathAndEventCommand. */
  public PathAndEventCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
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
