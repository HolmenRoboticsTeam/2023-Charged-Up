// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class FieldSim {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final Field2d m_field2d;
  private Pose2d[] m_swerveModulePoses = {
    new Pose2d(),
    new Pose2d(),
    new Pose2d(),
    new Pose2d()
  };

  public FieldSim(DrivetrainSubsystem drivetrainSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_field2d = new Field2d();

    SmartDashboard.putData("Field", this.m_field2d);
  }

  public void initSim() {}

  public Field2d getField2d() {
    return this.m_field2d;
  }

  private void updateRobotPose() {
    this.m_field2d.setRobotPose(this.m_drivetrainSubsystem.getPose());

    for (int i = 0; i < DrivetrainConstants.kModuleTranslations.length; i++) {
      Translation2d updatedPosition = DrivetrainConstants.kModuleTranslations[i]
          .rotateBy(this.m_drivetrainSubsystem.getPose().getRotation())
          .plus(this.m_drivetrainSubsystem.getPose().getTranslation());
      this.m_swerveModulePoses[i] = new Pose2d(
        updatedPosition,
        this.m_drivetrainSubsystem
          .getModule(DrivetrainSubsystem.SwerveModule.toEnum(i))
          .getState().angle
          .plus(this.m_drivetrainSubsystem.getHeading())
        );
    }

    this.m_field2d.getObject("Swerve Modules").setPoses(this.m_swerveModulePoses);
  }

  public void periodic() {
    this.updateRobotPose();
    if (RobotBase.isSimulation()) this.simulationPeriodic();
  }

  public void simulationPeriodic() {}

}

