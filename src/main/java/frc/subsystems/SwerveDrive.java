package frc.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriverControl;
import frc.utils.RobotPreferences;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDrive extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriverControl.kMaxDriveMeterS * 0.05).withRotationalDeadband(DriverControl.kMaxAngularRadS * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private Vision vision;
    private boolean shouldUseLimelight = false;
    private Pose2d robotPose;
    public Pose2d getRobotPose() {
        return robotPose;
    }

    private final Field2d field = new Field2d();

    private boolean blueAlliance = true;

    //Has to be in its own function, because of the template code
    private void intialization() {
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        //Telemetry
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putString(Constants.Swerve.ModuleNames[i] + "/.type", "SwerveModuleState");
        }
        this.registerTelemetry((SwerveDrivetrain.SwerveDriveState state) -> {
            robotPose = state.Pose;
            field.setRobotPose(state.Pose);
            SmartDashboard.putData("Field", field);

            for (int i = 0; i < state.ModuleStates.length; i++) {
                SmartDashboard.putNumber(Constants.Swerve.ModuleNames[i] + "/actualAngle", state.ModuleStates[i].angle.getDegrees());
                SmartDashboard.putNumber(Constants.Swerve.ModuleNames[i] + "/actualSpeed", state.ModuleStates[i].speedMetersPerSecond);
                SmartDashboard.putNumber(Constants.Swerve.ModuleNames[i] + "/setAngle", state.ModuleTargets[i].angle.getDegrees());
                SmartDashboard.putNumber(Constants.Swerve.ModuleNames[i] + "/setSpeed", state.ModuleTargets[i].speedMetersPerSecond);
            }
        });

        //Remote Commands
        SmartDashboard.putData("Zero Pose", new InstantCommand(() -> this.seedFieldRelative(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)))).withName("Zero Pose").ignoringDisable(true));
        
        SmartDashboard.putData("Reset to LL", new InstantCommand(() -> this.seedFieldRelative(
            vision.getBotPose2d())).withName("Reset to LL").ignoringDisable(true));
        
        vision = Vision.getInstance();
    }

    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        intialization();
    }
    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        intialization();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = m_moduleLocations[0].getNorm();
        
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * @return a Command that takes a SwerveRequest supplier and applies it for as long as
     * this command runs. 
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * @return Get the current robot-centric chassis speeds, directly from the module's actual
     * states.
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Sets the preferences of this subsystem, mainly whether or not to use the limelight
     * pose inputs.
     */
    public void setPreferences() {
        shouldUseLimelight = RobotPreferences.shouldUseLimelight();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) blueAlliance = false;
        }
    }

    @Override
    public void periodic() {
        // System.out.println(robotPose.getTranslation());

        if (shouldUseLimelight) {
            if (vision.couldSeeApriltag()) {
                //addVisionMeasurement(vision.getBotPose2d(),
                //    Timer.getFPGATimestamp());
            }
        }
        if (blueAlliance) {
            // System.out.println("To blue: " + this.robotPose.getTranslation().getDistance(
            //     Constants.FieldMeasurements.BlueSpeakerLocation));
        } else {
            // System.out.println("To red: " + this.robotPose.getTranslation().getDistance(
            //     Constants.FieldMeasurements.RedSpeakerLocation));
        }
    }
}
