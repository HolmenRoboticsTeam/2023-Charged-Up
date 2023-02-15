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
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveModuleConstants;
// import frc.robot.commands.ControlledHeadingDriveCommand; // Original
import frc.robot.commands.ControlledHeadingDriveCommand2;
import frc.robot.commands.DefaultDriveCommand2;
// import frc.robot.commands.DefaultDriveCommand; // Original
// import frc.robot.simulation.FieldSim; // Original
import frc.robot.simulation.FieldSim2;
// import frc.robot.subsystems.DrivetrainSubsystem; // Original
import frc.robot.subsystems.DriveTrainSubsystem2;
import frc.robot.subsystems.GripperSubsystem;


public class RobotContainer {
  private final DriveTrainSubsystem2 m_DriveTrainSubsystem2;
  private final XboxController m_driveController;
  private final ControlledHeadingDriveCommand2 m_controlledHeadingDriveCommand2;
  private final DefaultDriveCommand2 m_defaultDriveCommand;
  private final FieldSim2 m_fieldSim;
  private final GripperSubsystem m_gripperSubsystem;
  private final XboxController m_armController;

  private final JoystickButton m_armCompEnableButton;
  private final JoystickButton m_armCompDisableButton;

  private final JoystickButton m_armOpenGripperButton;
  private final JoystickButton m_armCloseGripperButton;
// Original 
  // private final DrivetrainSubsystem m_drivetrainSubsystem;
  // private final XboxController m_driveController;
  // private final ControlledHeadingDriveCommand m_controlledHeadingDriveCommand;
  // private final DefaultDriveCommand m_defaultDriveCommand;
  // private final FieldSim m_fieldSim;

  public RobotContainer() {
    this.m_DriveTrainSubsystem2 = new DriveTrainSubsystem2();
    this.m_gripperSubsystem = new GripperSubsystem();
    this.m_driveController = new XboxController(OIConstants.kDriveControllerPort);
    this.m_armController = new XboxController(OIConstants.kArmControllerPort);
    this.m_controlledHeadingDriveCommand2 = new ControlledHeadingDriveCommand2(this.m_driveController, this.m_DriveTrainSubsystem2);
    this.m_defaultDriveCommand = new DefaultDriveCommand2(this.m_driveController, this.m_DriveTrainSubsystem2);
    this.m_fieldSim = new FieldSim2(this.m_DriveTrainSubsystem2);
    

    
    this.m_armCompEnableButton = new JoystickButton(this.m_armController, XboxController.Button.kA.value);
    this.m_armCompDisableButton = new JoystickButton(this.m_armController, XboxController.Button.kB.value);

    this.m_armOpenGripperButton = new JoystickButton(this.m_armController, XboxController.Button.kRightBumper.value);
    this.m_armCloseGripperButton = new JoystickButton(this.m_armController, XboxController.Button.kLeftBumper.value);
  // Original
    // this.m_drivetrainSubsystem = new DrivetrainSubsystem();
    // this.m_driveController = new XboxController(OIConstants.kdriveControllerPort);
    // this.m_controlledHeadingDriveCommand = new ControlledHeadingDriveCommand(this.m_driveController, this.m_drivetrainSubsystem);
    // this.m_defaultDriveCommand = new DefaultDriveCommand(this.m_driveController, this.m_drivetrainSubsystem);
    // this.m_fieldSim = new FieldSim(this.m_drivetrainSubsystem);

    configureBindings();



    this.m_fieldSim.initSim();
    // this.m_drivetrainSubsystem.setDefaultCommand(this.m_defaultDriveCommand);
    this.m_DriveTrainSubsystem2.setDefaultCommand(this.m_defaultDriveCommand);

  }

  private void configureBindings() {
  this.m_armCompEnableButton.onTrue(new RunCommand(this.m_gripperSubsystem::compEnable, this.m_gripperSubsystem));
  this.m_armCompDisableButton.onTrue(new RunCommand(this.m_gripperSubsystem::compDisable, this.m_gripperSubsystem));
  this.m_armOpenGripperButton.onTrue(new RunCommand(this.m_gripperSubsystem::open, this.m_gripperSubsystem));
  this.m_armCloseGripperButton.onTrue(new RunCommand(this.m_gripperSubsystem::close, this.m_gripperSubsystem));
  }


  public Command buildSwerveAutoBuilder(List<PathPlannerTrajectory> pathGroup, HashMap<String, Command> eventMap) {

    //Creates the autobuilder. Only needs to be done once.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this.m_DriveTrainSubsystem2::getPose,             // Pose2d supplier
      this.m_DriveTrainSubsystem2::setPose,             // Pose2d consumer, used to reset odometry at the beginning of auto
      DrivetrainConstants.kDriveKinematics,            // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
      AutonomousConstants.kPIDTranslationAuto,         // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutonomousConstants.kPIDRotationAuto,            // PID constants to correct for rotation error (used to create the rotation controller)
      this.m_DriveTrainSubsystem2::setModuleStates,     // Module states consumer used to output to the drive subsystem
      eventMap,
      true,                          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this.m_DriveTrainSubsystem2);                     // The drive subsystem. Used to properly set the requirements of path following commands

      return autoBuilder.fullAuto(pathGroup);
  }
// Original
  //   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  //     this.m_drivetrainSubsystem::getPose,             // Pose2d supplier
  //     this.m_drivetrainSubsystem::setPose,             // Pose2d consumer, used to reset odometry at the beginning of auto
  //     DrivetrainConstants.kDriveKinematics,            // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
  //     AutonomousConstants.kPIDTranslationAuto,         // PID constants to correct for translation error (used to create the X and Y PID controllers)
  //     AutonomousConstants.kPIDRotationAuto,            // PID constants to correct for rotation error (used to create the rotation controller)
  //     this.m_drivetrainSubsystem::setModuleStates,     // Module states consumer used to output to the drive subsystem
  //     eventMap,
  //     true,                          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  //     this.m_drivetrainSubsystem);                     // The drive subsystem. Used to properly set the requirements of path following commands

  //     return autoBuilder.fullAuto(pathGroup);
  // }

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
// Blue-Load2-CrossAuto
  public Command blueLoad2CrossAuto() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load2-CrossAuto", AutonomousConstants.kPathConstraintsAuto);
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
    eventMap.put("Moving Path1", new PrintCommand("Passed marker 1"));
    eventMap.put("Moving Path2", new PrintCommand("Passed marker 2"));

      return this.buildSwerveAutoBuilder(pathGroup, eventMap);
    }
  public void periodic() {
    this.m_fieldSim.periodic();
  }

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
