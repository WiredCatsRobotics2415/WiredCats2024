package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.generated.TunerConstants;
import frc.robot.Constants.DriverControl;
import frc.subsystems.Vision;
import frc.subsystems.Intake;

/**
 * Automatically drives towards and intakes a note.
 */
public class AutoNoteDetect extends Command {
    //GENERAL
    private Vision vision = Vision.getInstance();
    private Intake intake = Intake.getInstance();

    //SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(DriverControl.kMaxDriveMeterS * 0.05)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public void intake() {
        intake.in();
       TunerConstants.DriveTrain.setControl(driveHeading.withVelocityX(0.5));
    }

    @Override
    public void initialize() {
        //Swerve preparations
        driveHeading.HeadingController = Constants.Swerve.headingPIDController;

        //if there's no note, end
        if (!vision.isNoteVisible()) {
            end(true);
        }
    }

    @Override
    public void execute() {
        //checks x-degree of note
        if (vision.getNoteAngleOnX() != 0) {
            //turns robot to current pose + x-degree
            Rotation2d pose = TunerConstants.DriveTrain.getRobotPose().getRotation();
            TunerConstants.DriveTrain.setControl(driveHeading
                .withTargetDirection(pose.plus(Rotation2d.fromDegrees(vision.getNoteAngleOnX()))));

            //start intake, end when IR sensor is passed
            intake();
            if (intake.hasNote()) {
                end(false);
            }
        } else {
            intake();
            if (intake.hasNote()) {
                end(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.off();
        TunerConstants.DriveTrain.setControl(driveHeading.withVelocityX(0));
        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}