// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.AutoConstants;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    this.configureButtonBindings();

    // Configure default commands
    this.m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> this.m_robotDrive.drive(
          -MathUtil.applyDeadband(this.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
            Rotation2d.fromRadians(ArmPositionConstants.kHomeAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

    this.m_pickUpFromFloorButton.onTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPlaceOnFloorBoomLength,
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceOnFloorAngle)
          )
        ),
        this.m_armSubsystem
      )
    );

    this.m_pickUpFromDriverStationButton.onTrue(
      new RunCommand(
        () -> this.m_armSubsystem.setDesiredState(
          new SwerveModulePosition(
            ArmPositionConstants.kPickUpFromDriverStationBoomLength,
            Rotation2d.fromRadians(ArmPositionConstants.kPickUpFromDriverStationAngle)
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
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceOnFloorAngle)
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
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceCubeOnLevel1Angle)
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
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceCubeOnLevel2Angle)
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
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceConeOnLevel1Angle)
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
            Rotation2d.fromRadians(ArmPositionConstants.kPlaceConeOnLevel2Angle)
          )
        ),
        this.m_armSubsystem
      )
    );

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
