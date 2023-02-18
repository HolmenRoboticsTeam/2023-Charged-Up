// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// // Original Code 



// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.OIConstants;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class DefaultDriveCommand extends CommandBase {
//   /** Creates a new ControllerCommand. */

//   private final XboxController m_controller;
//   private final DrivetrainSubsystem m_drivetrainSubsystem;

  
//   public DefaultDriveCommand(XboxController controller, DrivetrainSubsystem drivetrainSubsystem) {
//     this.m_controller = controller;
//     this.m_drivetrainSubsystem = drivetrainSubsystem;
//     this.addRequirements(m_drivetrainSubsystem);
    
//     // Use addRequirements() here to declare subsystem dependencies.
//   }
  
//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double throttle = MathUtil.applyDeadband(-this.m_controller.getLeftY(), OIConstants.kcontrollerDeadband);
//     double strafe = MathUtil.applyDeadband(-this.m_controller.getLeftX(), OIConstants.kcontrollerDeadband);
//     double rotation =  MathUtil.applyDeadband(this.m_controller.getRightX(), OIConstants.kcontrollerDeadband);
//     this.m_drivetrainSubsystem.drive(throttle, strafe, rotation, true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     this.m_drivetrainSubsystem.drive(0, 0, 0, true);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
