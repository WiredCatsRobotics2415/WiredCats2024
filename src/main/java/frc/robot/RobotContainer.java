package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.commands.FixAll;
import frc.commands.HotspotGenerator;
import frc.generated.TunerConstants;
import frc.robot.Constants.Drive;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Arm;
import frc.subsystems.Climber;
import frc.subsystems.Intake;
import frc.subsystems.Flywheel;
import frc.subsystems.Vision;

public class RobotContainer {
    private static RobotContainer instance;

    //SWERVE
    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain; //Use the already constructed instance
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Drive.kMaxDriveMeterS * 0.05).withRotationalDeadband(Drive.kMaxAngularRadS * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    //SUBSYSTEMS
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance(); 
    private final Flywheel flywheel = Flywheel.getInstance(); 
    private final Arm arm = Arm.getInstance();
    
    // HOTSPOT
    private final HotspotGenerator hotspotGen = HotspotGenerator.getInstance(); 

    //PUBLIC OBJECTS
    private OIs.OI selectedOI;
    public OIs.OI getSelectedOI() {
        return selectedOI;
    }

    //CHOOSERS
    private SendableChooser<Command> autoChooser;

    private RobotContainer() {
        // Configure auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("Auto"); 
        Shuffleboard.getTab("Auto")
            .add("Auto Chooser", autoChooser)
            .withSize(4, 2);

        // SmartDashboard.putData("Auto Chooser", autoChooser);
        arm.setGoal(arm.getMeasurement());
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Adds all Commands to the Triggers in the selectedOI's binds map.
     * Intended to be run in teleopInit.
     */
    private void configureButtonBindings() {
        //Swerve
        selectedOI.binds.get("PigeonReset").onTrue(new InstantCommand(() -> {
            swerveDrive.seedFieldRelative();
        }, swerveDrive));

        //Intake
        selectedOI.binds.get("Intake").onTrue(intake.toggleIntake());
        selectedOI.binds.get("ManualOuttake").onTrue(intake.out()).onFalse(intake.off());

        //Arm manual
        selectedOI.binds.get("RaiseArm").whileTrue(arm.increaseGoal());
        selectedOI.binds.get("LowerArm").whileTrue(arm.decreaseGoal()); 

        //Flywheel
        selectedOI.binds.get("Shoot").onTrue(
            intake.uptake().andThen(new WaitCommand(1)).andThen(intake.off()));
        // selectedOI.binds.get("SpinUp").onTrue(flywheel.toggleSpinedUp()); 
        selectedOI.binds.get("SpinUp").onTrue(flywheel.onFromSmartDashboard()); 
        // selectedOI.binds.get("SpinUp").onTrue(flywheel.on(Constants.Flywheel.FLYWHEEL_SPEED, Constants.Flywheel.FLYWHEEL_SPEED)); 

        //Climber 
        selectedOI.binds.get("LeftClimberDown").onTrue(
            climber.manualDown(Constants.Climber.ClimberSpeed, 0) 
        );
        selectedOI.binds.get("LeftClimberUp").onTrue(
            climber.manualUp(Constants.Climber.ClimberSpeed, 0) 
        );
        selectedOI.binds.get("RightClimberDown").onTrue(
            climber.manualDown(0, Constants.Climber.ClimberSpeed) 
        );
        selectedOI.binds.get("RightClimberUp").onTrue(
            climber.manualUp(0, Constants.Climber.ClimberSpeed)
        );
        selectedOI.binds.get("SpinOff").onTrue(flywheel.off()); 

        // Automatic
        selectedOI.binds.get("TargetHotspot").onTrue(new InstantCommand(() -> 
             CommandScheduler.getInstance().schedule(hotspotGen.targetClosest())
        ));

        selectedOI.binds.get("AmpPreset").onTrue(new InstantCommand(() -> {
            arm.setGoalInDegrees(81);
        }));

        // selectedOI.binds.get("TargetHotspot").onTrue(new FixAll());
    }

    /**
     * Adds all binds to triggers.
     * Intended to be run in teleopInit.
     */
    private void configureTriggers() {
        Trigger infraredTrigger1 = new Trigger(intake::hasNote);
        infraredTrigger1.onTrue(intake.queueNote());

        Trigger infraredTrigger2 = new Trigger(intake::noteIsQueued);
        infraredTrigger2.onTrue(intake.stopNoteForShooting());
        
    }

    /**
     * Calls methods from subsystems to update from preferences.
     * Intended to be run in teleopInit.
     */
    private void configurePreferences() {
        selectedOI.setPreferences();
        Vision.getInstance().setPreferences();
        swerveDrive.setPreferences();
    }

    /**
     * Prepares the robot for teleoperated control.
     * Gets the OI selected, configures all binds, and calls any teleopInit
     * methods on subsystems. CLEARS ALL DEFAULT EVENTLOOP BINDS
     */
    public void teleopInit() {
        //Turn everything "off"
        flywheel.off().schedule();
        intake.off().schedule();
        arm.setGoal(arm.getMeasurement());

        switch (OI.oiChooser.getSelected()) {
            case 0:
              selectedOI = new OIs.GulikitController();
              break;
            default:
              selectedOI = new OIs.GulikitController();
              break;
        }
        Robot.buttonEventLoop.clear();
        configurePreferences();
        configureButtonBindings();
        //configureTriggers();
        swerveDrive.setDefaultCommand(swerveDrive.applyRequest(() -> {
            TwoDControllerInput input = selectedOI.getXY();
            return drive.withVelocityX(-input.x() * Drive.kMaxDriveMeterS) // Drive forward with
                .withVelocityY(-input.y() * Drive.kMaxDriveMeterS) // Drive left with negative X (left)
                .withRotationalRate(-selectedOI.getRotation() * Drive.kMaxAngularRadS); // Drive counterclockwise with negative X (left)
            }
        ));

        //Subsystem enables
        flywheel.teleopInit();
    }
    
    /**
     * Gets the autonomous command to be run from Robot.java
     * @return
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        //return Autos.exampleAuto(m_exampleSubsystem);
        // return AutoBuilder.followPath(path); 
        // Test autonomous path 
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Very_Basic"); 
        String chosenAuto = autoChooser.getSelected().getName();
        return new PathPlannerAuto(chosenAuto); 
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
