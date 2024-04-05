package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriverControl;
import frc.generated.TunerConstants;
import frc.subsystems.Vision;
import frc.subsystems.Intake;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

/**
 * Automatically drives towards and intakes a note.
 */
public class AutoNoteDetect extends Command {
    //GENERAL
    private Vision vision = Vision.getInstance();
    private Intake intake = Intake.getInstance();

    private boolean isNoteVisible = true;

    //SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric()
        .withVelocityX(0.5 * DriverControl.kMaxDriveMeterS);
    
    public AutoNoteDetect() {
        addRequirements(TunerConstants.DriveTrain);
    }

    @Override
    public void initialize() {
        //Swerve preparations
        driveHeading.HeadingController = Constants.Swerve.headingPIDController;

        if (!vision.isNoteVisible()) {
            Logger.log(this, LogLevel.INFO, "No note visible on init");
        }
        Logger.log(this, LogLevel.INFO, "Auto note detect running");
        intake.intakeNote().schedule();
    }

    @Override
    public void execute() {
        if (vision.isNoteVisible()) {
        //turns robot to current pose + x-degree
        Rotation2d pose = TunerConstants.DriveTrain.getRobotPose().getRotation();
        System.out.println(vision.getNoteAngleOnX());

        if(vision.getNoteAngleOnX() >= Constants.Swerve.HeadingControllerTolerance ||
            vision.getNoteAngleOnX() <= -Constants.Swerve.HeadingControllerTolerance) {
            System.out.println("head on");

            //System.out.println(pose);
            //System.out.println(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())));

            TunerConstants.DriveTrain.setControl(driveHeading
                .withTargetDirection(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())))
            );
        } else {
            TunerConstants.DriveTrain.setControl(driveForward);
        }
        }
    }

    @Override
    public void end(boolean interrupted) {
        TunerConstants.DriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
        intake.off().schedule();
        Logger.log(this, LogLevel.INFO, "Ended", interrupted);
    }

    @Override
    public boolean isFinished() {
        //return false;
        Logger.log(this, LogLevel.INFO, vision.getNoteAngleOnX());
        return (intake.hasNoteIntakingOrNot() || !vision.isNoteVisible()); 
    }
}