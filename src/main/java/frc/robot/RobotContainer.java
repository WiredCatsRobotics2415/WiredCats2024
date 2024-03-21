package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.commands.AutoNoteDetect;
import frc.commands.ShootingPresets;
import frc.commands.ShootingPresets.Settings;
import frc.generated.TunerConstants;
import frc.subsystems.Finger;
import frc.robot.Constants.DriverControl;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;
import frc.subsystems.SwerveDrive;
import frc.subsystems.Arm;
import frc.subsystems.Climber;
import frc.subsystems.Intake;
import frc.subsystems.Flywheel;
import frc.subsystems.Vision;
import frc.utils.DriverFeedback;

public class RobotContainer {
    private static RobotContainer instance;

    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain;
    private final Intake intake = Intake.getInstance();
    //private final Climber climber = Climber.getInstance();
    private final Flywheel flywheel = Flywheel.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Finger finger = Finger.getInstance();

    // PUBLIC OBJECTS
    private OIs.OI selectedOI;

    // LOAD SHOOTING PRESETS
    private final ShootingPresets shooterPre = new ShootingPresets();

    //LOAD SPEAKER LOCATION
    public static Translation2d speakerLocation;

    public OIs.OI getSelectedOI() {
        return selectedOI;
    }

    // CHOOSERS
    private SendableChooser<Command> autoChooser;

    private RobotContainer() {
        // Configure auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("Auto");
        Shuffleboard.getTab("Auto")
                .add("Auto Chooser", autoChooser)
                .withSize(4, 2);

        // SmartDashboard.putData("Auto Chooser", autoChooser);
        neutralizeSubsystems();

        // Autonomous named commands
        NamedCommands.registerCommand("Intake", intake.intakeNote());
        NamedCommands.registerCommand("ShootSub", shooterPre.subwooferAuto()); // Shoot next to subwoofer. 
        NamedCommands.registerCommand("ShootWhileMoving", shooterPre.shootWhileMoving()); // Shoot next to subwoofer. 
        NamedCommands.registerCommand("FlywheelOn", flywheel.on(Settings.subwoofer.left_flywheel, Settings.subwoofer.right_flywheel)); // Shoot next to subwoofer. 
        NamedCommands.registerCommand("FlywheelOff", flywheel.off()); // Shoot next to subwoofer. 
        NamedCommands.registerCommand("Amp", shooterPre.shootAmp()); // Score in Amp.  
        NamedCommands.registerCommand("ShootMiddle", shooterPre.shootMiddle()); // Score in Amp. 
        NamedCommands.registerCommand("ShootBottom", shooterPre.shootBottom()); // Score in Amp. 
        NamedCommands.registerCommand("shootTop", shooterPre.shootTop()); // Score in Amp. 
         NamedCommands.registerCommand("ArmDown", arm.moveDown()); // Score in Amp.   
        //TODO: add in commands for shooting and dropping notes

        configureStartupTriggers();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Triggers intended to be run on startup
     */
    private void configureStartupTriggers() {
        new Trigger(RobotController::getUserButton).onTrue(arm.coast().ignoringDisable(true));
    }

    /**
     * Prepares the robot for teleoperated control.
     * Gets the OI selected, configures all binds, and calls any teleopInit
     * methods on subsystems. CLEARS ALL DEFAULT EVENTLOOP BINDS
     */
    public void teleopInit() {
        neutralizeSubsystems();
        prepareOI();
        configurePreferences();
        configureButtonBindings();
        configureTriggers();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue)
                speakerLocation = Constants.FieldMeasurements.BlueSpeakerLocation;
            else
                speakerLocation = Constants.FieldMeasurements.RedSpeakerLocation;
        } else {
            speakerLocation = Constants.FieldMeasurements.BlueSpeakerLocation; // Default to blue
        }
    }

    /**
     * Schedules flywheel and intake off, sets arm goal to current position
     */
    private void neutralizeSubsystems() {
        flywheel.off().schedule();
        intake.off().schedule();
        arm.setGoal(arm.getMeasurement());
        arm.brake();
    }

    /**
     * Sets the selected OI variable to the smartdashboard selected OI.
     * Intended to be run in teleopInit.
     */
    private void prepareOI() {
        switch (OI.oiChooser.getSelected()) {
            case 0:
                selectedOI = new OIs.GulikitController();
                break;
            default:
                selectedOI = new OIs.GulikitController();
                break;
        }
    }

    /**
     * Adds all Commands to the Triggers in the selectedOI's binds map, clears previous binds
     * Intended to be run in teleopInit.
     */
    private void configureButtonBindings() {
        Robot.buttonEventLoop.clear();

        // Swerve
        swerveDrive.setDefaultCommand(swerveDrive.applyRequest(() -> {
            TwoDControllerInput input = selectedOI.getXY();
            return swerveDrive.drive.withVelocityX(-input.x() * DriverControl.kMaxDriveMeterS)
                    .withVelocityY(-input.y() * DriverControl.kMaxDriveMeterS)
                    .withRotationalRate(-selectedOI.getRotation() * DriverControl.kMaxAngularRadS);
        }));
        selectedOI.binds.get("PigeonReset").onTrue(new InstantCommand(() -> {
            swerveDrive.seedFieldRelative();
        }, swerveDrive));

        // Intake
        selectedOI.binds.get("Intake").onTrue(intake.toggleIntake());
        //selectedOI.binds.get("Intake").onTrue(intake.intakeNote());
        selectedOI.binds.get("ManualOuttake").onTrue(intake.out()).onFalse(intake.off());
        //selectedOI.binds.get("ManualIntake").onTrue(intake.in()).onFalse(intake.off());

        // Arm manual
        selectedOI.binds.get("RaiseArm").whileTrue(arm.increaseGoal());
        selectedOI.binds.get("LowerArm").whileTrue(arm.decreaseGoal());

        // Fire 
        //selectedOI.binds.get("Shoot").onTrue(new ConditionalCommand(finger.fire(), new InstantCommand(() -> {}), flywheel::withinSetGoal));
        selectedOI.binds.get("Shoot").onTrue(finger.fire());
        //selectedOI.binds.get("ReverseFinger").whileTrue(new RepeatCommand(finger.reverse()));
        selectedOI.binds.get("ReverseFinger").onTrue(finger.reverse());
        
        // Flywheel 
        //TODO: change call to onFromSmartDashboard to a call to on(flwyheelSppeds)
        selectedOI.binds.get("ShootClose").onTrue(flywheel.onFromSmartDashboard());
        selectedOI.binds.get("SpinOff").onTrue(flywheel.off());
        selectedOI.binds.get("SpinUpToAmp").onTrue(flywheel.on(3000, 3000));

        // Climber
        // selectedOI.binds.get("LeftClimberDown").onTrue(
        //         climber.manualDown(Constants.Climber.ClimberSpeed, 0));
        // selectedOI.binds.get("LeftClimberUp").onTrue(
        //         climber.manualUp(Constants.Climber.ClimberSpeed, 0));
        // selectedOI.binds.get("RightClimberDown").onTrue(
        //         climber.manualDown(0, Constants.Climber.ClimberSpeed));
        // selectedOI.binds.get("RightClimberUp").onTrue(
        //         climber.manualUp(0, Constants.Climber.ClimberSpeed));

        // Presets

        selectedOI.binds.get("Amp").onTrue(shooterPre.shootAmp());
        selectedOI.binds.get("ArmDrivePreset").onTrue(new InstantCommand(() -> {
            arm.setGoal(5);
        }));
        selectedOI.binds.get("ArmIntakePosition").onTrue(new InstantCommand(() -> {
            arm.setGoal(0);
        }));
        //selectedOI.binds.get("ShootClose").onTrue(flywheel.on(6000, 8000)); // Subwoofer
        // selectedOI.binds.get("TargetHotspot").onTrue(new FixAll());

        selectedOI.binds.get("AutoIntake").onTrue(new AutoNoteDetect());
    }

    /**
     * Adds all binds to triggers.
     * Intended to be run in teleopInit.
     */
    private void configureTriggers() {
        //new Trigger(intake::hasNote).onTrue(intake.queueNote());
        //new Trigger(intake::noteIsQueued).onTrue(intake.stopNoteForShooting());
        // new Trigger(intake::hasNote).onTrue(
        //     finger.preventNoteFromContactingNote().andThen(
        //     DriverFeedback.blinkInConfirmation())
        // );
        new Trigger(intake::hasNote).onTrue(
            DriverFeedback.blinkInConfirmation()
        );
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
     * @return autonomous command to be run from Robot.java
     */
    public Command getAutonomousCommand() {
        String chosenAuto = autoChooser.getSelected().getName();
        return new PathPlannerAuto(chosenAuto);
    }

    public static Pose2d getStartingPose(String autoName) {
        return new PathPlannerAuto(autoName).getStaringPoseFromAutoFile(autoName);
    }
}
