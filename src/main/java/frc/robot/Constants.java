// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
*        |                                     |                                 |                            |
*        |                                     |                                 |                            |
*        |_ _ _ _     _ _ _ _    _ _ _ _   _ _ | _ _       |_ _ _ _    _ _ _ _   |_ _ _ _     _ _ _ _     _ _ | _ _
*        |        |  |       |  |         |_ _ | _ _|      |          |       |  |       |   |       |   |_ _ | _ _|
*        |        |  |_ _ _ _|  |_ _ _ _       |           |          |       |  |       |   |       |        |
*        |        |  |                  |      |           |          |       |  |       |   |       |        |
*        |_ _ _ _ |  |_ _ _ _    _ _ _ _|      |           |          |_ _ _ _|  |_ _ _ _|   |_ _ _ _|        |
*/
package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class AutonomousConstants {
    public static final double kMaxVelocity = 1.25;                   //max velocity in 4 m/s
    public static final double kMaxAcceleration = 3.0;               //max acceleration in 3 m/s^2

    public static final HashMap<String, Command> eventMap = new HashMap<>();

    public static final PIDConstants kPIDTranslationAuto = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kPIDRotationAuto = new PIDConstants(0.0, 0.0, 0.0);

    public static final PathConstraints kPathConstraintsAuto = new PathConstraints(kMaxVelocity, kMaxAcceleration);
    public static final PathConstraints kPathConstraintsAutoFast = new PathConstraints(4.0, 3.0);
    public static final PathConstraints kPathConstraintsAutoMedium = new PathConstraints(2.4, 3.0);
    public static final PathConstraints kPathConstraintsAutoSlow = new PathConstraints(1.0, 3.0);

  }

  public static final class ArmConstants {
    // Motor Driving Parameters
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2.0 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // SPARK MAX CAN IDs
    public static final int kExtendCanId = 10;
    public static final int kPivotCanId = 11;

    // Calculations required for extension motor conversion factors and feed forward
    // Drum diameter is averaged between fully contracted and fully extended wraps of the rope
    public static final double kExtendMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kPivotMotorFreeSpeedRps = Neo550MotorConstants.kFreeSpeedRpm / 60;
    public static final double kDrumDiameterMeters = 0.0259; // Units.inchesToMeters(1.02);
    public static final double kDrumCircumferenceMeters = kDrumDiameterMeters * Math.PI;
    // Setting up motor reductions for extention and pivot. Using FreeSpeedRPS to calculate FF for the pivot motor.
    public static final double kExtendMotorReduction = 1.0;
    public static final double kPivotMotorReduction = (46.0 / 25.0) * 100.0;
    public static final double kPivotArmFreeSpeedRps = (kPivotMotorFreeSpeedRps * kDrumCircumferenceMeters)
        / kPivotMotorReduction;

    public static final double kExtendEncoderCPR = 8192.0;
    public static final double kExtendEncoderPositionFactor = kDrumCircumferenceMeters / kExtendEncoderCPR; // Meters
    public static final double kFullyContractedLengthMeters = 1.035;  // Middle of drum to tip of gripper (rope length); Units.inchesToMeters(40.75);
    public static final double kGripperTipToMiddleOffsetMeters = 0.083;  // Units.inchesToMeters(3.25);

    // Encoder is on the output shaft
    public static final double kPivotEncoderPositionFactor = 360.0;
    public static final double kInternalPivotEncoderPositionFactor = 360.0 / kPivotMotorReduction;
    public static final double kInternalPivotEncoderVelocityFactor = kInternalPivotEncoderPositionFactor / 60.0;
    public static final double kPivotEncoderVelocityFactor = kPivotEncoderPositionFactor / 60.0;

    public static final Rotation2d kPivotForwardLimit = Rotation2d.fromDegrees(100);
    public static final Rotation2d kPivotReverseLimit = Rotation2d.fromDegrees(-10);

    public static final double kPivotEncoderPositionPIDMinInput = 0;  // Radians
    public static final double kPivotEncoderPositionPIDMaxInput = kPivotEncoderPositionFactor;  // Radians

    public static final double kPivotRampRateInSeconds = 3.0;

    public static final double kExtendP = 1e-4;
    public static final double kExtendI = 0.0; //1e-6;
    public static final double kExtendD = 0;
    public static final double kExtendFF = 0.000156;
    public static final double kExtendMinOutput = -1;
    public static final double kExtendMaxOutput = 1;
    public static final double kExtendMaxVel = 6000.0; // rpm
    public static final double kExtendMaxAcc = 2000.0;
    public static final double kExtendMinVel = 0;
    public static final double kExtendAllowedError = 0;

    public static final double kPivotP = 1e-4;
    public static final double kPivotI = 0.0; //1e-6;
    public static final double kPivotD = 0;
    public static final double kPivotFF = 0.000156;
    public static final double kPivotMinOutput = -1;
    public static final double kPivotMaxOutput = 1;
    public static final double kPivotMaxVel = 4500.0; // rpm
    public static final double kPivotMaxAcc = 2000.0;
    public static final double kPivotMinVel = 0;
    public static final double kPivotAllowedError = 0;

    public static final IdleMode kExtendMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kPivotMotorIdleMode = IdleMode.kBrake;

    public static final int kExtendMotorCurrentLimit = 20;
    public static final int kPivotMotorCurrentLimit = 20;

    public static final double kMaxHorizontalDistanceOffRobot = 59.5;  // Units.inchesToMeters(59.5)
    public static final double kMinHorizontalDistanceOffRobot = 0.0;

    public static final boolean kExtendEncoderInverted = false;

  }
// Angle and length of arm depending on button pressed. These are not the correct values

  public static final class ArmPositionConstants {
  public static final double kHomeAngle = 2;//Units.degreesToRadians(0);
  public static final double kPickUpFromFloorAngle = 23;
  public static final double kPickUpFromDriverStationAngle = 90; //.degreesToRadians(90);
  public static final double kPlaceOnFloorAngle = 40;
  public static final double kPlaceCubeOnLevel1Angle = 100;
  public static final double kPlaceCubeOnLevel2Angle = 110;
  public static final double kPlaceConeOnLevel1Angle = 100;
  public static final double kPlaceConeOnLevel2Part1Angle = 80;
  public static final double kPlaceConeOnLevel2Part2Angle = 110;

  public static final double kHomeBoomLength = 0.0;
  public static final double kPickUpFromFloorBoomLength = 140.0;
  public static final double kPickUpFromDriverStationBoomLength = 20.0;
  public static final double kPlaceOnFloorBoomLength = 0;
  public static final double kPlaceCubeOnLevel1BoomLength = 25.0;
  public static final double kPlaceCubeOnLevel2BoomLength = 0;
  public static final double kPlaceConeOnLevel1BoomLength = 25.0;
  public static final double kPlaceConeOnLevel2BoomLength = 210.0;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0; // 4.8 IS MAX SPEED
    public static final double kMaxAngularSpeed = 2.0 * Math.PI; // radians per second
    public static final double kMaxAnguolarSpeedSquared = 2.0 * Math.PI; //radians per secondF
    public static final double kSwerveTargetingOffset = 630.0; // Degrees

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kDriveRampRateInSeconds = 1.0;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kHeadingP = 0.0085;
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.0;
    public static final TrapezoidProfile.Constraints kHeadingControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeed, kMaxAnguolarSpeedSquared
      );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;


    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;  // Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxillaryControllerPort = 1;
    public static final double kDriveDeadband = 0.15;
    public static final int kDriverJoystickPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class Neo550MotorConstants {
    public static final double kFreeSpeedRpm = 11710;
  }

  public static final class GripperConstants {
    public static final int kForwardChannel = 0;
    public static final int kReverseChannel = 1;
  }
}
