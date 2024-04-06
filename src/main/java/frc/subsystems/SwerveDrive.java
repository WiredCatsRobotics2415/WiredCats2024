package frc.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriverControl;
import frc.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.LimelightHelpers.RawFiducial;
import frc.utils.LimelightHelpers.Results;
import frc.utils.Logger.LogLevel;
import frc.utils.Logger;
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
            new SwerveRequest.ApplyChassisSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity);
    public final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriverControl.kMaxDriveMeterS * 0.05)
                    .withRotationalDeadband(
                            DriverControl.kMaxAngularRadS * 0.05) // Add a 5% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = 
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(DriverControl.kMaxDriveMeterS * 0.05);

    private Vision vision;
    private int visionMeasurmentCounter = 0;
    private boolean shouldUseLimelight = false;
    private SendableChooser<Boolean> useLimelightChooser = new SendableChooser<Boolean>();
    private Pose2d robotPose;
    private Pose2d lastLimelightPose = new Pose2d();

    public Pose2d getRobotPose() {
        return robotPose;
    }

    private final Field2d field = new Field2d();

    // Has to be in its own function, because of the template code
    private void intialization() {
        useLimelightChooser.setDefaultOption("Do not use", false);
        useLimelightChooser.addOption("Use", true);
        Shuffleboard.getTab("Auto")
                .add("Use apriltags?", useLimelightChooser)
                .withSize(2, 1);
        addExtraMotorConfigs();
        driveFacingAngle.HeadingController = Constants.Swerve.headingPIDController;
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
                new InstantCommand(() -> this.seedFieldRelative(vision.getShooterResults().pose))
                        .withName("Reset to LL")
                        .ignoringDisable(true));

        vision = Vision.getInstance();
    }

    private void addExtraMotorConfigs() {
        for (SwerveModule m : this.Modules) {
            m.getDriveMotor().getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.2));
        }
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
                        new PIDConstants(0.1, 0.0, 0), // Translation PID constants
                        new PIDConstants(0.1, 0.0, 0), // Rotation PID constants
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

    public Command faceAngle(Rotation2d angle) {
        Command command = new Command() {
            public void execute() {
                TunerConstants.DriveTrain.setControl(TunerConstants.DriveTrain.driveFacingAngle
                        .withTargetDirection(angle));
            };

            @Override
            public boolean isFinished() {
                double rotation = TunerConstants.DriveTrain.getRobotPose().getRotation().getDegrees();
                System.out.println(rotation);
                System.out.println(angle.getDegrees() - 20);
                System.out.println(angle.getDegrees() + 20);
                return (rotation >= angle.getDegrees() - 20 &&
                        rotation <= angle.getDegrees() + 20);
            }
        };
        command.addRequirements(this);
        return command;
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
        //shouldUseLimelight = useLimelightChooser.getSelected();
        // shouldUseLimelight = DriverStation.isTeleop();
        if (shouldUseLimelight) Logger.log(this, LogLevel.INFO, "USING LIMELIGHT");
        else Logger.log(this, LogLevel.INFO, "NOT USING LIMELIGHT");
    }

    /**
     * Adds the shooter limelight measurements to the odometer Adapted from
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
     */
    private void processMegatag() {
        PoseEstimate lastResults = vision.getShooterResults();
        //if (lastResults.tagCount == 0) return;
        
        Pose2d recievedPose = lastResults.pose;
        Pose2d deadReckPose = this.m_odometry.getEstimatedPosition();

        double poseDifference = deadReckPose.getTranslation()
                .getDistance(recievedPose.getTranslation());
        double lastLimelightPoseDistance = lastLimelightPose.getTranslation()
                .getDistance(recievedPose.getTranslation());
        
        Logger.log(this, LogLevel.INFO, poseDifference, lastLimelightPoseDistance, lastResults.avgTagDist);
        if (poseDifference > 1.0d || lastLimelightPoseDistance > 1.0d || lastResults.avgTagDist > 1.5d) {
            lastLimelightPose = recievedPose;
            return;
        }
        lastLimelightPose = recievedPose;

        double bestTargetArea = 0.0d;
        for (RawFiducial aprilTag : lastResults.rawFiducials) {
            if (aprilTag.ta > bestTargetArea) bestTargetArea = aprilTag.ta;
        }
        if (bestTargetArea <= 0.1d) return;

        double xyStds = 1.0d;
        // multiple targets detected
        if (lastResults.rawFiducials.length >= 2) {
            xyStds = 0.5;
        }
        // 1 target
        if (lastResults.rawFiducials.length == 1) {
            xyStds = 0.75;
        }
        Pose2d poseToGive = new Pose2d(recievedPose.getTranslation(), deadReckPose.getRotation());
        double translationSpeed = getTranslationSpeed();
        if (translationSpeed >= 0.5 && translationSpeed <= 2.5) {
            m_odometry.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, 0.0d));
            m_odometry.addVisionMeasurement(
                poseToGive, Timer.getFPGATimestamp() - (lastResults.latency/1000.0));
            Logger.log(this, LogLevel.INFO, "Adding estimation", lastResults.rawFiducials.length, translationSpeed, poseToGive, deadReckPose, translationSpeed);
        }
    }

    private double getTranslationSpeed() {
        ChassisSpeeds currentChassisSpeeds = getCurrentRobotChassisSpeeds();
        double x = currentChassisSpeeds.vxMetersPerSecond;
        double y = currentChassisSpeeds.vyMetersPerSecond;
        return Math.sqrt((x*x)+(y*y));
    }

    @Override
    public void periodic() {
        if (shouldUseLimelight) {
            //processMegatag();
        }
    }
}

