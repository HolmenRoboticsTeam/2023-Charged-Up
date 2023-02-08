// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ArmSim {

    // private Mechanism2d m_mech2d = new Mechanism2d(5, 5);
    // private MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 4, 4);
    // the main mechanism object
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_extender =
      m_armPivot.append(
        new MechanismLigament2d("extender", 2, 90));

    public ArmSim() {
      SmartDashboard.putData("m_extendingMechanism", this.m_mech2d);
    }

    public Mechanism2d getMechanism2d() {
      return this.m_mech2d;
    }

    public void periodic() {

    }
    public void simulationPeriodic() {}
}


