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
import frc.subsystems.Intake;

/**
 * Automatically drives towards and intakes a note.
 */
public class AutoNoteDetect extends Command {
    //GENERAL
    private Vision vision = Vision.getInstance();
    private Intake intake = Intake.getInstance();

    private final double driveSpeed = 0.5;

    //SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public AutoNoteDetect() {
        addRequirements(TunerConstants.DriveTrain, intake);
    }

    @Override
    public void initialize() {
        //Swerve preparations
        driveHeading.HeadingController = Constants.Swerve.headingPIDController;

        if (!vision.isNoteVisible()) {
            Logger.log(this, LogLevel.INFO, "No note visible on init");
            return;
        }
        Logger.log(this, LogLevel.INFO, "Auto note detect running");
        intake.intakeNote().schedule();
    }

    @Override
    public void execute() {
        //turns robot to current pose + x-degree
        Rotation2d pose = TunerConstants.DriveTrain.getRobotPose().getRotation();
        // figure out the speeds that robot has to move at
        double driveX = Math.cos(pose.getRadians()) * driveSpeed;
        double driveY = Math.sin(pose.getRadians()) * driveSpeed;
        Logger.log(this, LogLevel.INFO, driveX, driveY);

        TunerConstants.DriveTrain.setControl(driveHeading
            .withTargetDirection(pose.plus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())))
            .withVelocityX(driveX)
            .withVelocityY(driveY)
        );
    }

    @Override
    public void end(boolean interrupted) {
        Logger.log(this, LogLevel.INFO, "Ended", interrupted);
        intake.off().schedule();
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}