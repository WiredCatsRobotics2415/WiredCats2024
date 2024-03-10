package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.subsystems.SwerveDrive;
import frc.generated.TunerConstants;
import frc.robot.Constants.DriverControl;
import frc.subsystems.Vision;
import frc.generated.TunerConstants;
import frc.subsystems.sensors.IR;
import frc.subsystems.Intake;

/**
 * The "FixAll" preset (FIX ALL subsystems to their ideal position for scoring).
 * Instance can be reused (ie. you can construct this command once for a button binding).
 * Automatically compensates for alliance.
 */
public class AutoNoteDetect extends Command {
    //GENERAL
    private RobotContainer robotContainer; //Cache instance
    private Vision vision;
    private IR ir;
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
        if (vision.getNote() == false) {
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
            if (ir.rightIR.getValue() > Constants.Intake.IRThreshold) {
                end(false);
            }
            //TODO: stop intake after 3 sec so doesn't cross line?
        } else {
            intake();
            if (ir.rightIR.getValue() > Constants.Intake.IRThreshold) {
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