package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.generated.TunerConstants;
import frc.subsystems.Vision;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

/**
 * Automatically drives towards and intakes a note.
 */
public class AutoNoteDetect extends Command {
    //GENERAL
    private Vision vision = Vision.getInstance();

    //SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public AutoNoteDetect() {
        addRequirements(TunerConstants.DriveTrain);
    }

    @Override
    public void initialize() {
        //Swerve preparations
        driveHeading.HeadingController = Constants.Swerve.headingPIDController;

        if (!vision.isNoteVisible()) {
            Logger.log(this, LogLevel.INFO, "No note visible on init");
            end(false);
        }
        Logger.log(this, LogLevel.INFO, "Auto note detect running");
    }

    @Override
    public void execute() {
        //turns robot to current pose + x-degree
        Rotation2d pose = TunerConstants.DriveTrain.getRobotPose().getRotation();

        TunerConstants.DriveTrain.setControl(driveHeading
            .withTargetDirection(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())))
        );
    }

    @Override
    public void end(boolean interrupted) {
        Logger.log(this, LogLevel.INFO, "Ended", interrupted);
    }

    @Override
    public boolean isFinished() {
        //return false;
        Logger.log(this, LogLevel.INFO, vision.getNoteAngleOnX());
        return (vision.getNoteAngleOnX() >= -Constants.Swerve.HeadingControllerTolerance &&
                vision.getNoteAngleOnX() <= Constants.Swerve.HeadingControllerTolerance); 
    }
}