// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
// Everything here is a final because they are constant. They should not change UNLESS we directly change it.
public final class Constants {

    public static final class AutonomousConstants {
        public static final int kMaxVelocity = 0;                   //max velocity of 4 m/s
        public static final int kMaxAcceleration = 0;               //a max acceleration of 3 m/s^2

        public static final HashMap<String, Command> eventMap = new HashMap<>();
    }


}
