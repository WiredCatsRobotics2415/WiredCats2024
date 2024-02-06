// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.generated.TunerConstants;

public final class Constants {
    public static class Swerve {
      public static final String[] ModuleNames = new String[] {
        "Front Left",
        "Front Right",
        "Back Left",
        "Back Right"
      };
    }

    public static class Drive {
      public static final double kMaxDriveMeterS = TunerConstants.kSpeedAt12VoltsMps;
      public static final double kMaxAngularRadS = Math.PI; //rad/second
    }

    /**
     * Translations in terms of blue origin (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin)
     * (The same as 0,0 on the Field2D widget on shuffleboard)
     */
    public static class FieldConstants {
      public static final Translation2d BlueSpeakerLocation = new Translation2d(
        Units.inchesToMeters(46),
        Units.inchesToMeters(236));
      public static final Translation2d RedSpeakerLocation = new Translation2d(
        Units.inchesToMeters(662),
        Units.inchesToMeters(236));
    }
}
