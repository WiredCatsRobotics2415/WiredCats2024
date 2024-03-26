// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.commands.Hotspot;
import frc.generated.TunerConstants;

import java.util.ArrayList;

public final class Constants {
    public static class Swerve {
        public static final String[] ModuleNames =
                new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};

        public static final PhoenixPIDController headingPIDController =
                new PhoenixPIDController(3, 0, 0);

        static {
            headingPIDController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public static final int HeadingControllerTolerance = 4;
    }

    public static class Climber {
        public static final double ClimberGearRatio =
                6.746031746031747; // Drive gear ratio - Testing
        public static final double ClimberMax =
                metersToRotations(1) * ClimberGearRatio; // Rotations
        public static final double ClimberMin =
                metersToRotations(0) * ClimberGearRatio; // Rotations
        public static final double ClimberSpeed = 0.25; // Open-duty cycle

        public static double metersToRotations(
                int rotations) { // TODO: Needs to be completed once climber is finished
            return rotations;
        }
    }

    public static class Flywheel {
        public static final Slot0Configs LEFT_PID =
                new Slot0Configs()
                        .withKV(0.14)
                        .withKP(0.25);
        public static final Slot0Configs RIGHT_PID =
                new Slot0Configs()
                        .withKV(0.14)
                        .withKP(0.2);

        public static final CurrentLimitsConfigs CURRENT_LIMITS =
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(80) // initial: 80
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30) // initial: 30
                        .withSupplyCurrentThreshold(60)
                        .withSupplyTimeThreshold(0.1);
        public static final MotorOutputConfigs COAST_CONFIG =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

        public static final double GOAL_TOLERANCE_RPM = 500;

        public static final double GEAR_RATIO = 3.4d;

        public static double rpsToRPM(double rps) {
            return (rps * 60) * GEAR_RATIO;
        }

        public static double rpmToRPS(double goalRPM) {
            return (goalRPM / 60) / GEAR_RATIO;
        }
    }

    public static class ShooterCalculations {
        public static double getCalculatedFlywheelVelocity() {
            double X = TunerConstants.DriveTrain.getRobotPose().getTranslation().getX();
            double Y = TunerConstants.DriveTrain.getRobotPose().getTranslation().getY();
            double R = Math.sqrt(X * X + Y * Y);

            double model = 0; // result in rpm

            return model;
        }

        public static double getCalculatedArmShooterAngle() {
            Translation2d speakerDist = RobotContainer.getInstance().getSpeakerLocation().minus(TunerConstants.DriveTrain.getRobotPose().getTranslation());

            double X = Units.metersToInches(speakerDist.getX());
            double Y = Units.metersToInches(speakerDist.getY());
            double R = Math.sqrt(X * X + Y * Y);

            double model = (-5.26 * Math.pow(10,-6))*Math.pow(R,3)+(-1.46 * Math.pow(10,-3))*Math.pow(R,2)+(0.487)*R+(-6.36); // result in degrees

            if(model > 0 ){
                return model;
            } else{
                return 0;
            }
        }

        public static double getCalculatedFlywheelSpin() {
            double X = TunerConstants.DriveTrain.getRobotPose().getTranslation().getX();
            double Y = TunerConstants.DriveTrain.getRobotPose().getTranslation().getY();
            double R = Math.sqrt(X * X + Y * Y);

            double model = 0; // result in ratio

            return model;
        }
    }

    public static class DriverControl {
        public static final double kMaxDriveMeterS = TunerConstants.kSpeedAt12VoltsMps;
        public static final double kMaxAngularRadS = Math.PI * 1.5d; // rad/second
        public static final double MinimumDrivePower = 0.05d;
    }

    public static class Intake {
        public static final double UptakeSpeed = 1;
        public static final double IntakeSpeed = 0.5;
        public static final double OuttakeSpeed = -0.35;
        public static final double IRThreshold = 100;
    }

    public static class Finger {
        public static final double FINGER_GEAR_RATIO = 12; // 20:1 gear ratio
        public static final double DISTANCE = 1.25; // rotations

        public static final double Ks = 0.14;
        // public static final double Kp = 3.0;
        // public static final double Kd = 0.01;
        public static final double Kp = 1.0;
        public static final double Kd = 0.005;
        public static final double outputExtrema = 0.83;
    }

    public static final ArrayList<Hotspot> Hotspots = new ArrayList<Hotspot>();

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

        public static final double POT_OFFSET =
                0.4727; // In ROTATIONS (added before reading is converted to rotations)

        public static final float KS = 0.238f;
        public static final float KV = 0.0f;
        public static final float KA = 0.0f;
        public static final float KG = 0.1f;
        public static final float KP = 0.18f;
        public static final float KD = 0.0033f;

        public static final float VELO_MAX = 300f;
        public static final float ACCEL_MAX = 300f; // 6

        public static final float ROTOR_TO_ARM_GEAR_RATIO =
                280 / 1; // (# encoder rotations per 1 full rotation)

        public static double rotationsToFalcon(double rotations) {
            return rotations * ROTOR_TO_ARM_GEAR_RATIO;
        }

        public static double falconToRotations(double rotations) {
            return rotations / ROTOR_TO_ARM_GEAR_RATIO;
        }

        public static double MAX_VOLT = 3.07d;
        public static double MIN_VOLT = 1.74d;
        public static final double MAX_ANGLE = 105.7d;
        public static final double MIN_DEGREES = 0.0d; // Min angle of arm
    }

    /**
     * Translations in terms of blue origin
     * (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin)
     * (The same as 0,0 on the Field2D widget on shuffleboard)
     */
    public static class FieldMeasurements {
        public static final Translation2d BlueSpeakerLocation =
                new Translation2d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(207)); // TODO: may need fudge factoring
        public static final Translation2d RedSpeakerLocation =
                new Translation2d(
                        Units.inchesToMeters(651.598),
                        Units.inchesToMeters(207)); // TODO: may need fudge factoring
    }

    public static class Vision {
        public static final String ShooterLimelightName = "limelight-back";
        public static final String IntakeLimelightName = "limelight-intake";
    }
}
