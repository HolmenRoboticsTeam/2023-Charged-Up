// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.compressor;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

/** Add your docs here. */
public class CompressorIOReal implements CompressorIO{
    private Compressor m_compressor;
    public CompressorIOReal(){
        this.m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }
    @Override
    public void updateInputs(CompressorIOInputs inputs) {
        // TODO Auto-generated method stub
        inputs.compressorIsEnabled = this.m_compressor.isEnabled();
        inputs.compressorIsOn = this.m_compressor.isEnabled();
    }
    @Override
    public void disable() {
        this.m_compressor.disable();
    }

    @Override
    public void enable() {
        this.m_compressor.enableDigital();
    }
}
