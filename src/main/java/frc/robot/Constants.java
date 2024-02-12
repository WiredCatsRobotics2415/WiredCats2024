// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

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

      public static final PhoenixPIDController headingPIDController = new PhoenixPIDController(10, 0, 0);
      static {
        headingPIDController.enableContinuousInput(-Math.PI, Math.PI);
      }
    }

    public static class Climber {
      public static final double ClimberGearRatio = 6.746031746031747; // Drive gear ratio - Testing  
      public static final double ClimberMax = Conversions.rotations_to_meters(1) * ClimberGearRatio; // Rotations 
    }

    public static class Conversions {
      public static double rotationsToMeters(int rotations) { // Needs to be completed once arm is finished
        return rotations; 
      }
    }

    public static class Drive {
      public static final double kMaxDriveMeterS = TunerConstants.kSpeedAt12VoltsMps;
      public static final double kMaxAngularRadS = Math.PI; //rad/second
      public static final double MinimumDrivePower = 0.05d;
    }

    /**
     * Translations in terms of blue origin (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin)
     * (The same as 0,0 on the Field2D widget on shuffleboard)
     */
    public static class FieldMeasurements {
      public static final Translation2d BlueSpeakerLocation = new Translation2d(
        Units.inchesToMeters(0),
        Units.inchesToMeters(207)); //TODO: may need fudge factoring
      public static final Translation2d RedSpeakerLocation = new Translation2d(
        Units.inchesToMeters(651.598),
        Units.inchesToMeters(207)); //TODO: may need fudge factoring
    }
}
