// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.Nothing;
import frc.robot.autonomous.testChargeStation;
import frc.robot.autonomous.Comp1;
import frc.robot.autonomous.Comp2;
import frc.robot.autonomous.Comp3;
import frc.robot.autonomous.Comp4;
import frc.robot.commands.ArmPivotToHomeCommand;
import frc.robot.commands.ArmRetractToHomeCommand;
import frc.robot.commands.ToggleCompressorStateCommand;
import frc.robot.commands.ToggleGripperStateCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.compressor.Compressor;
import frc.robot.subsystems.compressor.CompressorIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOReal;
import frc.robot.subsystems.gripper.GripperIOReal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Arm m_armSubsystem = new Arm(new ArmIOReal());
  private final Drive m_driveSubsystem = new Drive(new DriveIOReal());
  private final Gripper m_gripperSubsystem = new Gripper(new GripperIOReal());
  private final Compressor m_compressorSubystem = new Compressor(new CompressorIOReal());

  // The robot's commands
  private final ToggleGripperStateCommand m_toggleGripperStateCommand = new ToggleGripperStateCommand(m_gripperSubsystem);
  private final ToggleCompressorStateCommand m_toggleCompressorStateCommand = new ToggleCompressorStateCommand(m_compressorSubystem);
  private final SequentialCommandGroup m_armGoHomeCommand = new SequentialCommandGroup(new ArmRetractToHomeCommand(m_armSubsystem), new ArmPivotToHomeCommand(m_armSubsystem));

  // The robot's autonomous routines
  private final Nothing m_nothing = new Nothing(m_armSubsystem, m_driveSubsystem, m_gripperSubsystem);
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

    m_autoChooser.setDefaultOption("Charge Station Only", m_testChargeStation);
    m_autoChooser.addOption("Comp-1", m_comp1);
    m_autoChooser.addOption("Comp-2", m_comp2);
    m_autoChooser.addOption("Comp-3", m_comp3);
    m_autoChooser.addOption("Comp-4", m_comp4);
    m_autoChooser.addOption("Nothing", this.m_nothing);

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
