package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commands.ShootingPresets;
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

public class RobotContainer {
    private static RobotContainer instance;

    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain;
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Flywheel flywheel = Flywheel.getInstance();
    private final Arm arm = Arm.getInstance();
    // private final Finger finger = Finger.getInstance();

    // PUBLIC OBJECTS
    private OIs.OI selectedOI;

    // LOAD SHOOTING PRESETS
    private final ShootingPresets shooterPre = new ShootingPresets();

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
        NamedCommands.registerCommand("Intake", intake.intakeAuto());
        NamedCommands.registerCommand("ShootSub", shooterPre.subwooferAuto()); // Shoot next to subwoofer. 
        NamedCommands.registerCommand("Amp", shooterPre.shootAmp()); // Score in Amp.  
        //TODO: add in commands for shooting and dropping notes
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
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
    }

    /**
     * Schedules flywheel and intake off, sets arm goal to current position
     */
    private void neutralizeSubsystems() {
        flywheel.off().schedule();
        intake.off().schedule();
        arm.setGoal(arm.getMeasurement());
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
        selectedOI.binds.get("ManualOuttake").onTrue(intake.out()).onFalse(intake.off());
        //selectedOI.binds.get("ManualIntake").onTrue(intake.in()).onFalse(intake.off());

        // Arm manual
        selectedOI.binds.get("RaiseArm").whileTrue(arm.increaseGoal());
        selectedOI.binds.get("LowerArm").whileTrue(arm.decreaseGoal());

        // Fire 
        /* 
        selectedOI.binds.get("Shoot").onTrue(
                finger.fire());
        */ 
        
        // Presets 
        selectedOI.binds.get("SpinUp").onTrue(flywheel.onFromSmartDashboard()); // TESTING ONLY
        selectedOI.binds.get("ShootClose").onTrue(shooterPre.shootClose()); // Subwoofer
        selectedOI.binds.get("Amp").onTrue(shooterPre.shootAmp()); // Amp 

        // Climber
        selectedOI.binds.get("LeftClimberDown").onTrue(
                climber.manualDown(Constants.Climber.ClimberSpeed, 0));
        selectedOI.binds.get("LeftClimberUp").onTrue(
                climber.manualUp(Constants.Climber.ClimberSpeed, 0));
        selectedOI.binds.get("RightClimberDown").onTrue(
                climber.manualDown(0, Constants.Climber.ClimberSpeed));
        selectedOI.binds.get("RightClimberUp").onTrue(
                climber.manualUp(0, Constants.Climber.ClimberSpeed));
        selectedOI.binds.get("SpinOff").onTrue(flywheel.off());

        // Automatic
        // selectedOI.binds.get("Amp").onTrue(shooterPre.shootAmp());

        // selectedOI.binds.get("TargetHotspot").onTrue(new FixAll());
    }

    /**
     * Adds all binds to triggers.
     * Intended to be run in teleopInit.
     */
    private void configureTriggers() { //shouldn't be needed anymore with Sequential Command
        //new Trigger(intake::hasNote).onTrue(intake.queueNote());
        //new Trigger(intake::noteIsQueued).onTrue(intake.stopNoteForShooting());
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
}
