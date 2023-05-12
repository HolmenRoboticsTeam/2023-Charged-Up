// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmIOReal implements ArmIO{
    private Arm m_arm;
    public ArmIOReal(){
        this.m_arm = new Arm(null);
    }
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.boomAngle = this.m_arm.getAngle();
        inputs.boomLength = this.m_arm.getBoomLength();
    }

    @Override
    public Rotation2d getAngle() {
       return this.m_arm.getAngle();
      }

    @Override
    public double getBoomLength() {
        return this.m_arm.getBoomLength();
    }
}
