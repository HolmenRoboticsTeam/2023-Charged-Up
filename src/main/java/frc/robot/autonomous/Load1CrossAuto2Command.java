// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Load1CrossAuto2Command extends SequentialCommandGroup {
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final GripperSubsystem m_gripperSubsystem;

  /** Creates a new Load1CrossAuto2Command. */
  public Load1CrossAuto2Command(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, GripperSubsystem gripperSubsystem) {
    this.m_armSubsystem = armSubsystem;
    this.m_driveSubsystem = driveSubsystem;
    this.m_gripperSubsystem = gripperSubsystem;
    addCommands(this.loadPath());
  }

  public Command loadPath() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      "Load1-CrossAuto2",
      AutonomousConstants.kPathConstraintsAutoSlow,
      AutonomousConstants.kPathConstraintsAutoMedium,
      AutonomousConstants.kPathConstraintsAutoMedium


    );
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("closeGripper", this.closeGripper());
    eventMap.put("openGripper", this.openGripper());
    eventMap.put("pivotLevel1Cone", this.pivotLevel1Cone());
    eventMap.put("coneDropWait", this.coneDropWait());
    eventMap.put("pivotGroundPickup", this.pivotGroundPickup());
    eventMap.put("retractArmHome", this.retractArmHome());
    eventMap.put("stopInXPattern", this.stopInXPattern());

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this.m_driveSubsystem::getPose,             // Pose2d supplier
      this.m_driveSubsystem::setPose,             // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics,            // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
      AutonomousConstants.kPIDTranslationAuto,         // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutonomousConstants.kPIDRotationAuto,            // PID constants to correct for rotation error (used to create the rotation controller)
      this.m_driveSubsystem::setAutoModuleState,     // Module states consumer used to output to the drive subsystem
      eventMap,
      true,                          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this.m_driveSubsystem                    // The drive subsystem. Used to properly set the requirements of path following commands
    );

      return autoBuilder.fullAuto(pathGroup);
  }

  private Command closeGripper() {
    return new InstantCommand(this.m_gripperSubsystem::close, this.m_gripperSubsystem);
  }

  private Command openGripper() {
    return new InstantCommand(this.m_gripperSubsystem::open, this.m_gripperSubsystem);

  }

  private Command pivotLevel1Cone() {
    return new InstantCommand(
      () -> this.m_armSubsystem.setDesiredState(
        new SwerveModulePosition(
          ArmPositionConstants.kPlaceConeOnLevel1BoomLength,
          Rotation2d.fromDegrees(ArmPositionConstants.kPlaceConeOnLevel1Angle)
        )
      ),
      this.m_armSubsystem
    );
  }

  private Command coneDropWait() {
    return new WaitCommand(0.5);
  }

  private Command pivotGroundPickup() {
    return new InstantCommand(
      () -> this.m_armSubsystem.setDesiredState(
        new SwerveModulePosition(
          ArmPositionConstants.kPickUpFromFloorBoomLength,
          Rotation2d.fromDegrees(ArmPositionConstants.kPickUpFromFloorAngle)
        )
      ),
      this.m_armSubsystem
    );
  }

  private Command retractArmHome() {
    return new InstantCommand(
      () -> this.m_armSubsystem.setBoomLength(ArmPositionConstants.kHomeBoomLength),
      this.m_armSubsystem
    );
  }

  private Command stopInXPattern() {
    return new InstantCommand(
      this.m_driveSubsystem::setX,
      this.m_driveSubsystem
    );
  }

}
