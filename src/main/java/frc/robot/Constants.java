// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.generated.TunerConstants;

public final class Constants {
    public static final String CANBusName = "rio"; 

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
      public static final double ClimberMax = Conversions.rotationsToMeters(1) * ClimberGearRatio; // Rotations 
    }

    public static class Flywheel {
      public static final double FLYWHEEL_SPEED = rpm_to_rps(2000);  // Flywheel only acceps input in rotations per second (rps) but we are more comfortable with rpms
 
      public static double rpm_to_rps(double rpm) {
        return rpm / 60; 
      }
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

    public static final class Arm {
      public static enum EncoderOption {
          ANALOG_POT,
          FALCON_ROTOR
      }
      public static final EncoderOption ENCODER_TO_USE = EncoderOption.ANALOG_POT;
  
      public static final double POT_OFFSET = 0.0d; //In DEGREES (added before reading is converted to rotations)
  
      public static final float KS = 0.3159f;
      public static final float KV = 8.06f;
      public static final float KA = 0.09f;
      public static final float KG = 0.383f;
      public static final float KP = 3.2f;
      public static final float KD = 1.5f;
  
      public static final float KG_PROPORTION = 0.005f; //How much to modify KG by;
      //applied KG = (Proportion * angle in degrees) * KG 
  
      public static final float VELO_MAX = 0.5f; //No more than 45 deg per second
      public static final float ACCEL_MAX = 0.25f;
  
      public static final float ROTOR_TO_ARM_GEAR_RATIO = 100/1; //(# encoder rotations per 1 full rotation)
  
      public static final double MAX_ROTATIONS = 120/360.0; //Max angle of arm
      public static final double MIN_ROTATIONS = 0/360.0; //Min angle of arm
      
      public static double rotationsToFalcon(double rotations) {
        return rotations * ROTOR_TO_ARM_GEAR_RATIO;
      }
  
      public static double falconToRotations(double rotations) {
        return rotations / ROTOR_TO_ARM_GEAR_RATIO;
      }
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

    public static class Vision {
      public final static String ShooterLimelightName = "back";
      public final static String IntakeLimelightName = "intake";
    }
}
