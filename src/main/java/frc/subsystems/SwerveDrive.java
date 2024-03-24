package frc.subsystems;

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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import frc.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.utils.LimelightHelpers.Results;
import frc.utils.RobotPreferences;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class SwerveDrive extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest =
            new SwerveRequest.ApplyChassisSpeeds();
    public final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriverControl.kMaxDriveMeterS * 0.05)
                    .withRotationalDeadband(
                            DriverControl.kMaxAngularRadS * 0.05) // Add a 5% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private Vision vision;
    private boolean shouldUseLimelight = false;
    private Pose2d robotPose;

    public Pose2d getRobotPose() {
        return robotPose;
    }

    private final Field2d field = new Field2d();

    // Has to be in its own function, because of the template code
    private void intialization() {
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Telemetry
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putString(
                    Constants.Swerve.ModuleNames[i] + "/.type", "SwerveModuleState");
        }
        this.registerTelemetry(
                (SwerveDrivetrain.SwerveDriveState state) -> {
                    robotPose = state.Pose;
                    field.setRobotPose(state.Pose);
                    SmartDashboard.putData("Field", field);

                    for (int i = 0; i < state.ModuleStates.length; i++) {
                        SmartDashboard.putNumber(
                                Constants.Swerve.ModuleNames[i] + "/actualAngle",
                                state.ModuleStates[i].angle.getDegrees());
                        SmartDashboard.putNumber(
                                Constants.Swerve.ModuleNames[i] + "/actualSpeed",
                                state.ModuleStates[i].speedMetersPerSecond);
                        SmartDashboard.putNumber(
                                Constants.Swerve.ModuleNames[i] + "/setAngle",
                                state.ModuleTargets[i].angle.getDegrees());
                        SmartDashboard.putNumber(
                                Constants.Swerve.ModuleNames[i] + "/setSpeed",
                                state.ModuleTargets[i].speedMetersPerSecond);
                    }
                });

        // Remote Commands
        SmartDashboard.putData(
                "Zero Pose",
                new InstantCommand(
                                () ->
                                        this.seedFieldRelative(
                                                new Pose2d(0, 0, Rotation2d.fromDegrees(0))))
                        .withName("Zero Pose")
                        .ignoringDisable(true));

        SmartDashboard.putData(
                "Reset to LL",
                new InstantCommand(() -> this.seedFieldRelative(vision.getShooterResults().targetingResults.getBotPose2d_wpiBlue()))
                        .withName("Reset to LL")
                        .ignoringDisable(true));

        vision = Vision.getInstance();
    }

    public SwerveDrive(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        intialization();
    }

    public SwerveDrive(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        intialization();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = m_moduleLocations[0].getNorm();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                                                 // live in your Constants class
                        new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0, 0.0, 0.0), // Rotation PID constants
                        3, // Max module speed, in m/s
                        driveBaseRadius, // Drive base radius in meters. Distance from robot center
                                         // to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for
                                               // the options here
                        ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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
     * @return a Command that takes a SwerveRequest supplier and applies it for as long as this
     *     command runs.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * @return Get the current robot-centric chassis speeds, directly from the module's actual
     *     states.
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Sets the preferences of this subsystem, mainly whether or not to use the limelight pose
     * inputs.
     */
    public void setPreferences() {
        shouldUseLimelight = RobotPreferences.shouldUseLimelight();
    }

    /**
     * Adds the shooter limelight measurements to the odometer Adapted from
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
     */
    private void processMegatag() {
        Results lastResults = vision.getShooterResults().targetingResults;
        Pose2d recievedPose = lastResults.getBotPose2d_wpiBlue();
        Pose2d deadReckPose = this.m_odometry.getEstimatedPosition();

        if (recievedPose.getX() == 0.0) {
            return;
        }

        double poseDifference = deadReckPose.getTranslation()
                .getDistance(recievedPose.getTranslation());

        double bestTargetArea = 0.0d;
        for (LimelightTarget_Fiducial aprilTag : lastResults.targets_Fiducials) {
            if (aprilTag.ta > bestTargetArea) bestTargetArea = aprilTag.ta;
        }

        if (lastResults.valid) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (lastResults.targets_Fiducials.length >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (bestTargetArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (bestTargetArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            Pose2d poseToGive = new Pose2d(recievedPose.getTranslation(), deadReckPose.getRotation());
            m_odometry.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            m_odometry.addVisionMeasurement(
                    poseToGive, Timer.getFPGATimestamp() - (lastResults.botpose[6]/1000.0));
        }
    }

    @Override
    public void periodic() {
        if (shouldUseLimelight) {
            processMegatag();
        }
    }
}
