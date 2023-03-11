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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  private final CompressorSubsystem m_compressorSubystem = new CompressorSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  // The robot's commands
  private final ToggleGripperStateCommand m_toggleGripperStateCommand = new ToggleGripperStateCommand(m_gripperSubsystem);
  private final ToggleCompressorStateCommand m_toggleCompressorStateCommand = new ToggleCompressorStateCommand(m_compressorSubystem);
  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_Joystick2 = new Joystick(OIConstants.kDriverJoystickPort);
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

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_Joystick2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Camera settings, self explanatory
    m_cameraTwo.setResolution(320, 240);
    m_cameraTwo.setFPS(30);

    // Configure the button bindings
    this.configureButtonBindings();
    this.m_compressorSubystem.disable();

    // Configure default commands
    this.m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> this.m_robotDrive.drive(
          MathUtil.applyDeadband(this.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(this.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),  // XBOX One controller, Right X-axis is ID 3 -- WPILib getRightX will not work (id 4)
          true,
          true
        ),
        this.m_robotDrive
      )
    );
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
    new JoystickButton(this.m_driverController, Button.kR1.value)
      .whileTrue(
        new RunCommand(
          () -> this.m_robotDrive.setX(),
          this.m_robotDrive
        )
      );
    this.m_toggleGripperButton.toggleOnTrue(this.m_toggleGripperStateCommand);
    this.m_toggleCompressorButton.toggleOnTrue(this.m_toggleCompressorStateCommand);

    this.m_homeButton.onTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kHomeBoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kHomeAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

    this.m_pickUpFromFloorButton.onTrue(
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

    this.m_pickUpFromDriverStationButton.onTrue(
      new InstantCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPickUpFromDriverStationBoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPickUpFromDriverStationAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

  this.m_placeOnFloorButton.onTrue(
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

  this.m_placeCubeOnLevel1Button.onTrue(
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

  this.m_placeCubeOnLevel2Button.onTrue(
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

  this.m_placeConeOnLevel1Button.onTrue(
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

  this.m_placeConeOnLevel2Button.onTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceConeOnLevel2BoomLength,
            Rotation2d.fromDegrees(ArmPositionConstants.kPlaceConeOnLevel2Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

  }

  public Command buildSwerveAutoBuilder(List<PathPlannerTrajectory> pathGroup, HashMap<String, Command> eventMap) {

    //Creates the autobuilder. Only needs to be done once.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this.m_robotDrive::getPose,             // Pose2d supplier
      this.m_robotDrive::setPose,             // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics,            // SwerveDriveKinematics ***UNDEFINED AT THE MOMENT 1/30/2023***
      AutonomousConstants.kPIDTranslationAuto,         // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutonomousConstants.kPIDRotationAuto,            // PID constants to correct for rotation error (used to create the rotation controller)
      this.m_robotDrive::setAutoModuleState,     // Module states consumer used to output to the drive subsystem
      eventMap,
      true,                          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this.m_robotDrive);                     // The drive subsystem. Used to properly set the requirements of path following commands

      return autoBuilder.fullAuto(pathGroup);
  }

  public Command blueLoad1CrossAndBalanceAuto() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue-Load1-CrossAuto-ChargeStation", AutonomousConstants.kPathConstraintsAuto);
    HashMap<String, Command> eventMap = new HashMap<>();

    /** Creates a new PathAndEventSubsystem. */
    eventMap.put("placeCone", new PrintCommand("Passed place cone"));
    eventMap.put("setNewArmPosition", new PrintCommand("arm is going up"));
    eventMap.put("setArmGroundPosition", new PrintCommand("arm is going down"));
    eventMap.put("takeCone", new PrintCommand("Passed take cone"));

    return this.buildSwerveAutoBuilder(pathGroup, eventMap);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Load 1, cross, and balance", blueLoad1CrossAndBalanceAuto());
    // m_chooser.addOption("Load 3, cross", blueLoad3CrossAuto());
    // m_chooser.addOption("Load 2, cross", blueLoad2CrossAuto());
    SmartDashboard.putData(m_chooser);

    return m_chooser.getSelected();
  }
}
