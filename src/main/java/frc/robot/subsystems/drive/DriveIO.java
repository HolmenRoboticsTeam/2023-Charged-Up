// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double heading;
        
        public double[] driveMotorTemp = {0.0, 0.0, 0.0, 0.0};
        public double[] driveMotorCurrentDraw = {0.0, 0.0, 0.0, 0.0};
        public double[] turningMotorTemp = {0.0, 0.0, 0.0, 0.0};
        public double[] turningMotorCurrentDraw = {0.0, 0.0, 0.0, 0.0};
    }

    public default void updateOdometry() {}
    public default void updateInputs(DriveIOInputs inputs) {}
    public default Pose2d getPose() {return null;}
    public default Pose2d getPoseAuto() {return null;}
    public default void setPose(Pose2d pose) {}
    public default void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {}
    public default void headingDrive(double throttle, double strafe, double desiredAngle, boolean fieldRelative) {}
    public default void headingDrive(double throttle, double strafe, double xRotation, double yRotation, boolean fieldRelative) {}

    public default void setX() {}
    public default void setModuleStates(SwerveModuleState[] desiredStates) {}
    public default void setAutoModuleState(SwerveModuleState[] desiredStates) {}
    public default void resetEncoders() {}
    public default void zeroHeading() {}
    public default double getHeading() {return 0.0;}
    public default double getPitch() {return 0.0;}
    public default double getRoll() {return 0.0;}
    public default double getTurnRate() {return 0.0;}
}
