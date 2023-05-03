// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.GripperConstants;

/** Add your docs here. */
public class GripperIOReal implements GripperIO {

    private final DoubleSolenoid m_valve;

    public GripperIOReal() {
      this.m_valve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GripperConstants.kForwardChannel, GripperConstants.kReverseChannel);
      this.m_valve.set(Value.kOff);
    }
    
    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.isGripperOpen = this.m_valve.get() == Value.kReverse;
    }


    @Override
    public void open() {
      this.m_valve.set(Value.kReverse);

    }

    @Override
    public void close() {
      this.m_valve.set(Value.kForward);
    }


}
