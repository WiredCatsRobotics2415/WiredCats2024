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
import frc.generated.TunerConstants;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;
import frc.subsystems.SwerveDrive;
import frc.robot.Constants.Drive;

public class RobotContainer {
    //SWERVE
    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain; //Use the already constructed instance
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Drive.kMaxDriveMeterS * 0.05).withRotationalDeadband(Drive.kMaxAngularRadS * 0.05) // Add a 2% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    //OBJECTS
    private OIs.OI selectedOI;

    //SMARTDASHBOARD
    private SendableChooser<Command> autoChooser; //TODO: Add auto chooser
    private final Field2d field = new Field2d();

    public RobotContainer() {
        // Configure auto chooser
        //autoChooser = AutoBuilder.buildAutoChooser("Basic_Auto"); 
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDrive.registerTelemetry((SwerveDrivetrain.SwerveDriveState state) -> {
            field.setRobotPose(state.Pose);
            SmartDashboard.putData("Field", field);
        });
    }

    private void configButtonBindings() {
        selectedOI.binds.get("navX Reset").onTrue(new InstantCommand(() -> {
            swerveDrive.seedFieldRelative();
        }, swerveDrive));
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

    /*
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
