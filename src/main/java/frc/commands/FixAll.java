package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverControl;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

/**
 * The "FixAll" preset (FIX ALL subsystems to their ideal position for scoring).
 * Instance can be reused (ie. you can construct this command once for a button binding).
 * Automatically compensates for alliance.
 */
public class FixAll extends Command {
    //GENERAL
    private RobotContainer robotContainer = RobotContainer.getInstance();

    // SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriverControl.kMaxDriveMeterS * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private Rotation2d goalHeading;

    public FixAll() {
        addRequirements(TunerConstants.DriveTrain); // TODO: add other subsystems
    }

    @Override
    public void initialize() {
        Translation2d speakerDist = robotContainer.getSpeakerLocation().minus(TunerConstants.DriveTrain.getRobotPose().getTranslation());
        goalHeading = Rotation2d.fromRadians(
                Math.atan2(speakerDist.getY(), speakerDist.getX())).plus(Rotation2d.fromDegrees(180));

        driveHeading.HeadingController = Constants.Swerve.headingPIDController;
    }

    @Override
    public void execute() {
        // Heading alignment
        // let (x, y) be the difference of position between the speaker and the robot
        // swerveheading = (arctan(y/x))

        TunerConstants.DriveTrain.setControl(driveHeading
                .withTargetDirection(goalHeading));
    }

    @Override
    public boolean isFinished() {
        double currentRotation = TunerConstants.DriveTrain.getRobotPose().getRotation().getRotations();
        return (currentRotation + Constants.Swerve.HeadingControllerTolerance < goalHeading.getDegrees()) && 
                (currentRotation - Constants.Swerve.HeadingControllerTolerance > goalHeading.getDegrees());
    }
}
