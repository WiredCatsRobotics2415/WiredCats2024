package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.commands.FixAll;
import frc.generated.TunerConstants;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Climber;
import frc.subsystems.Intake;
import frc.robot.Constants.Drive;

public class RobotContainer {
    private static RobotContainer instance;

    //SWERVE
    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain; //Use the already constructed instance
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Drive.kMaxDriveMeterS * 0.05).withRotationalDeadband(Drive.kMaxAngularRadS * 0.05) // Add a 2% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    //SUBSYSTEMS
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance(); 

    //PUBLIC OBJECTS
    private OIs.OI selectedOI;
    public OIs.OI getSelectedOI() {
        return selectedOI;
    }

    private Pose2d robotPose;
    public Pose2d getRobotPose() {
        return robotPose;
    }

    //SMARTDASHBOARD
    private SendableChooser<Command> autoChooser; //TODO: Add auto chooser
    private final Field2d field = new Field2d();

    private RobotContainer() {
        // Configure auto chooser
        //autoChooser = AutoBuilder.buildAutoChooser("Basic_Auto"); 
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        //TELEMETRY SETUP
        //Do .type ahead of time to avoid constant resend
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putString(Constants.Swerve.ModuleNames[i] + "/.type", "SwerveModuleState");
        }
        swerveDrive.registerTelemetry((SwerveDrivetrain.SwerveDriveState state) -> {
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

        ConfigSmartdashboard();
    }

    // Add all Smartdashboard widgets here 
    private void ConfigSmartdashboard() {
        climber.DisplayClimberPos(); 
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void configButtonBindings() {
        selectedOI.binds.get("PigeonReset").onTrue(new InstantCommand(() -> {
            swerveDrive.seedFieldRelative();
        }, swerveDrive));
        selectedOI.binds.get("FixAll").whileTrue(new FixAll()); //TODO: may need reconstruction each time?
        selectedOI.binds.get("Intake").whileTrue(new InstantCommand(() -> intake.toggle()));
        selectedOI.binds.get("ReleaseClimber").whileTrue(climber.release()); 
        selectedOI.binds.get("RetractClimber").onTrue(climber.retract()); 
    }

    //Calls methods from subsystems to update from preferences
    private void configurePreferences() {
        selectedOI.setPreferences();
    }

    public void teleopInit() {
        //Gets the OI selected
        switch (OI.oiChooser.getSelected()) {
            case 0:
              selectedOI = new OIs.GulikitController();
              break;
            default:
              selectedOI = new OIs.GulikitController();
              break;
        }
        configurePreferences();
        configButtonBindings();
        swerveDrive.setDefaultCommand(swerveDrive.applyRequest(() -> {
            TwoDControllerInput input = selectedOI.getXY();
            return drive.withVelocityX(-input.x() * Drive.kMaxDriveMeterS) // Drive forward with
                .withVelocityY(-input.y() * Drive.kMaxDriveMeterS) // Drive left with negative X (left)
                .withRotationalRate(-selectedOI.getRotation() * Drive.kMaxAngularRadS); // Drive counterclockwise with negative X (left)
            }
        ));
    }
     
    public Command getAutonomousCommand() {
        // Test autonomous path 
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Very_Basic"); 

        // An example command will be run in autonomous
        //return Autos.exampleAuto(m_exampleSubsystem);
        // return AutoBuilder.followPath(path); 
        return new PathPlannerAuto("Auto");
    }

    /**
    * Test pathfinding 
    * Robot starts at (1, 5) and ends at (3, 7): 2 x 2m
    */
    public Command getPathfindingCommand() {
        swerveDrive.seedFieldRelative(new Pose2d(2, 5, Rotation2d.fromDegrees(180))); //degrees:180
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(4, 7, Rotation2d.fromDegrees(0));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }
}
