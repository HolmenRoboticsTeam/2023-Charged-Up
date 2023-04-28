// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.Load1CrossAutoChargeStationAutoCommand;
import frc.robot.autonomous.Load1CrossChargeStationCrossAuto1;
import frc.robot.autonomous.Load2CrossAuto1;
import frc.robot.autonomous.Nothing;
import frc.robot.autonomous.testChargeStation;
import frc.robot.autonomous.Comp1;
import frc.robot.autonomous.Comp2;
import frc.robot.autonomous.Comp3;
import frc.robot.autonomous.Comp4;
import frc.robot.autonomous.CrossAuto;
import frc.robot.autonomous.CrossAuto2;
import frc.robot.autonomous.Load1;
import frc.robot.autonomous.Load1ChargeStation1;
import frc.robot.autonomous.Load1CrossAuto1Command;
import frc.robot.autonomous.Load1CrossAuto2Command;
import frc.robot.commands.ArmExtendToConeLevel2Command;
import frc.robot.commands.ArmPivotToHomeCommand;
import frc.robot.commands.ArmRetractToHomeCommand;
import frc.robot.commands.ToggleCompressorStateCommand;
import frc.robot.commands.ToggleGripperStateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final UsbCamera m_cameraTwo = CameraServer.startAutomaticCapture();
  // The robot's subsystems
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final CompressorSubsystem m_compressorSubystem = new CompressorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  // The robot's commands
  private final ToggleGripperStateCommand m_toggleGripperStateCommand = new ToggleGripperStateCommand(m_gripperSubsystem);
  private final ToggleCompressorStateCommand m_toggleCompressorStateCommand = new ToggleCompressorStateCommand(m_compressorSubystem);
  private final SequentialCommandGroup m_armGoHomeCommand = new SequentialCommandGroup(new ArmRetractToHomeCommand(m_armSubsystem), new ArmPivotToHomeCommand(m_armSubsystem));

  // The robot's autonomous routines
  private final Load1ChargeStation1 m_load1ChargeStation1 = new Load1ChargeStation1(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load1CrossAuto1Command m_load1CrossAutoCommand = new Load1CrossAuto1Command(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load1CrossAuto2Command m_load1CrossAuto2Command = new Load1CrossAuto2Command(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load1CrossAutoChargeStationAutoCommand m_load1CrossAutoChargeStationAutoCommand = new Load1CrossAutoChargeStationAutoCommand(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  // private final Load1CrossChargeStationCrossAuto m_load1CrossChargeStationCrossAuto = new Load1CrossChargeStationCrossAuto(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load1CrossChargeStationCrossAuto1 m_load1CrossChargeStationCrossAuto1 = new Load1CrossChargeStationCrossAuto1(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load2CrossAuto1 m_load2CrossAuto1 = new Load2CrossAuto1(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Load1 m_load1 = new Load1(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Nothing m_nothing = new Nothing(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final CrossAuto m_crossAuto = new CrossAuto(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final CrossAuto2 m_crossAuto2 = new CrossAuto2(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);

  private final Comp1 m_comp1 = new Comp1(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Comp2 m_comp2 = new Comp2(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Comp3 m_comp3 = new Comp3(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final Comp4 m_comp4 = new Comp4(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
  private final testChargeStation m_testChargeStation = new testChargeStation(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);


  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_Joystick2 = new Joystick(OIConstants.kDriverJoystickPort);
  private final JoystickButton m_setXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  // private final Joystick m_auxillaryJoystick = new Joystick(OIConstants.kAuxillaryControllerPort);
  private final JoystickButton m_toggleGripperButton = new JoystickButton(m_Joystick2, 12);
  private final JoystickButton m_toggleCompressorButton = new JoystickButton(m_Joystick2, 8);
  private final JoystickButton m_homeButton = new JoystickButton(m_Joystick2, 1);
  private final JoystickButton m_pickUpFromFloorButton = new JoystickButton(m_Joystick2, 2);
  private final JoystickButton m_pickUpFromDriverStationButton = new JoystickButton(m_Joystick2, 3);
  private final JoystickButton m_placeOnFloorButton = new JoystickButton(m_Joystick2, 4);
  private final JoystickButton m_placeCubeOnLevel1Button = new JoystickButton(m_Joystick2, 5);
  private final JoystickButton m_placeCubeOnLevel2Button = new JoystickButton(m_Joystick2, 6);
  private final JoystickButton m_placeConeOnLevel1Button = new JoystickButton(m_Joystick2, 7);
  private final JoystickButton m_placeConeOnLevel2Button = new JoystickButton(m_Joystick2, 9);
  private final JoystickButton m_slowDriveMode = new JoystickButton(m_driverController, 5);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Camera settings, self explanatory
    m_cameraTwo.setResolution(176, 144);
    m_cameraTwo.setFPS(30);

    // Configure the button bindings
    this.configureButtonBindings();

    // Configure default commands
    this.m_driveSubsystem.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> this.m_driveSubsystem.headingDrive(
          -MathUtil.applyDeadband(this.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(this.m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(this.m_driverController.getRawAxis(5), OIConstants.kDriveDeadband),
          true
        ),
        this.m_driveSubsystem
      )
    );

    this.m_armSubsystem.setDefaultCommand(this.m_armGoHomeCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    this.m_slowDriveMode.whileTrue(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> this.m_driveSubsystem.headingDrive(
            -MathUtil.applyDeadband(this.m_driverController.getLeftY(), OIConstants.kDriveDeadband) * 0.25,
            -MathUtil.applyDeadband(this.m_driverController.getLeftX(), OIConstants.kDriveDeadband) * 0.25,
            MathUtil.applyDeadband(this.m_driverController.getRawAxis(4), OIConstants.kDriveDeadband) * 0.25,
            MathUtil.applyDeadband(this.m_driverController.getRawAxis(5), OIConstants.kDriveDeadband) * 0.25,
            true
          ),
          this.m_driveSubsystem
        )
      );
    this.m_toggleGripperButton.toggleOnTrue(this.m_toggleGripperStateCommand);
    this.m_toggleCompressorButton.toggleOnTrue(this.m_toggleCompressorStateCommand);

    this.m_homeButton.onTrue(this.m_armGoHomeCommand);

    this.m_pickUpFromFloorButton.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPickUpFromFloorBoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPickUpFromFloorAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

    this.m_pickUpFromDriverStationButton.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPickUpFromDriverStationBoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPickUpFromDriverStationAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeOnFloorButton.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceOnFloorBoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceOnFloorAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeCubeOnLevel1Button.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceCubeOnLevel1BoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceCubeOnLevel1Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeCubeOnLevel2Button.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceCubeOnLevel2BoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceCubeOnLevel2Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeConeOnLevel1Button.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceConeOnLevel1BoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceConeOnLevel1Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeConeOnLevel2Button.whileTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceConeOnLevel2BoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceConeOnLevel2Part2Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

    this.m_setXButton.toggleOnTrue(
      new RunCommand(this.m_driveSubsystem::setX, this.m_driveSubsystem)
    );
    // m_autoChooser.setDefaultOption("Load 1 Cross Auto 1", this.m_load1CrossAutoCommand);
    // m_autoChooser.addOption("Load 1, cross, and balance", this.m_load1CrossAutoChargeStationAutoCommand);
    // m_autoChooser.addOption("Load 2 Cross Auto 1", this.m_load2CrossAuto1);
    // m_autoChooser.addOption("Load 1 Cross Auto 2", this.m_load1CrossAuto2Command);
    // m_autoChooser.addOption("Load 1 Charge Station 1", this.m_load1ChargeStation1);
    // m_autoChooser.addOption("Load 2 Cross Auto Charge Station 1", this.m_load1CrossChargeStationCrossAuto1);
    // m_autoChooser.addOption("Load1", this.m_load1);
    // m_autoChooser.addOption("Cross Auto", this.m_crossAuto);
    // m_autoChooser.addOption("Cross Auto 2", this.m_crossAuto2);

    m_autoChooser.setDefaultOption("Charge Station Only", m_testChargeStation);
    m_autoChooser.addOption("Comp-1", m_comp1);
    m_autoChooser.addOption("Comp-2", m_comp2);
    m_autoChooser.addOption("Comp-3", m_comp3);
    m_autoChooser.addOption("Comp-4", m_comp4);
    m_autoChooser.addOption("Nothing", this.m_nothing);


    // m_chooser.addOption("Load 3, cross", blueLoad3CrossAuto());
    // m_chooser.addOption("Load 2, cross", blueLoad2CrossAuto());
    SmartDashboard.putData(m_autoChooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
