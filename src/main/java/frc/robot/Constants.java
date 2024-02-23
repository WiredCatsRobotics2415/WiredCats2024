// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.commands.Hotspot;
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
      public static final double ClimberMax = metersToRotations(1) * ClimberGearRatio; // Rotations 
      public static final double ClimberMin = metersToRotations(0) * ClimberGearRatio; // Rotations
      public static final double ClimberSpeed = 0.25; // Open-duty cycle 

      public static double metersToRotations(int rotations) { // Needs to be completed once climber is finished
        return rotations; 
      }
    }

    public static class Flywheel {
      public static final Slot0Configs LEFT_PID = new Slot0Configs()
        .withKV(0.16)
        .withKP(0.24);
    
      public static final Slot0Configs RIGHT_PID = new Slot0Configs()
        .withKV(0.16)
        .withKP(0.24);

      public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.1);
      
      public static final MotorOutputConfigs COAST_CONFIG = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
      
      public static final double GOAL_TOLERANCE_RPM = 250;
      
      public static final double GEAR_RATIO = 3.4d;
      
      public static double rpsToRPM(double rps) {
        return (rps * 60) * GEAR_RATIO;
      }

      public static double rpmToRPS(double goalRPM) {
        return (goalRPM / 60) / GEAR_RATIO; // Account for gear ratio --> The speed which the motors have to run at to achieve goal rpm of wheels
      }
    }

    /**
     * Constants for teleoperated control.
     */
    public static class Drive {
      public static final double kMaxDriveMeterS = TunerConstants.kSpeedAt12VoltsMps;
      public static final double kMaxAngularRadS = Math.PI*1.5d; //rad/second
      public static final double MinimumDrivePower = 0.05d;
    }

    public static class Intake {
      public static final double UptakeSpeed = 1;
      public static final double IntakeSpeed = 0.9; 
      public static final double OuttakeSpeed = -0.4; 
      public static final double IRThreshold = 200; 
    }

    public final static ArrayList<Hotspot> Hotspots = new ArrayList<Hotspot>();
    static {
        Hotspots.add(new Hotspot(136.5, 200)); 
        Hotspots.add(new Hotspot(155, 236.5)); 
        Hotspots.add(new Hotspot(136.5, 243.5)); 
        Hotspots.add(new Hotspot(241.5, 295)); 
        Hotspots.add(new Hotspot(241.5, 238)); 
        Hotspots.add(new Hotspot(241.5, 181)); 
        Hotspots.add(new Hotspot(194.5, 112.638)); 
        Hotspots.add(new Hotspot(194.5, 324)); 
    }

    public static final class Arm {
      public static enum EncoderOption {
          ANALOG_POT,
          FALCON_ROTOR
      }
      public static final EncoderOption ENCODER_TO_USE = EncoderOption.ANALOG_POT;
  
      public static final double POT_OFFSET = 0.42; //In ROTATIONS (added before reading is converted to rotations)
  
      public static final float KS = 0.293f;
      public static final float KV = 0.09f;
      public static final float KA = 0.018f;
      public static final float KG = 0.1f;
      public static final float KP = 30f;
      public static final float KD = 0.9f;
  
      public static final float KG_PROPORTION = 0.1f; //How much to modify KG by;
      //applied KG = (Proportion * angle in degrees) * KG 
  
      public static final float VELO_MAX = 0.5f; //No more than 45 deg per second
      public static final float ACCEL_MAX = 0.25f;
  
      public static final float ROTOR_TO_ARM_GEAR_RATIO = 280/1; //(# encoder rotations per 1 full rotation)
  
      public static final double MAX_ROTATIONS = 60/360.0; //Max angle of arm
      public static final double MIN_ROTATIONS = 0.0; //Min angle of arm
      
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
      public final static String ShooterLimelightName = "limelight-back";
      public final static String IntakeLimelightName = "intake";
    }
}
