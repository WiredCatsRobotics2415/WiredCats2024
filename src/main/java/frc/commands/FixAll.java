package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drive;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;

public class FixAll extends Command {
    //GENERAL
    private OI oi; //Cache instance
    private RobotContainer robotContainer; //Cache instance

    // SWERVE
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Drive.kMaxDriveMeterS * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private PhoenixPIDController headingController = new PhoenixPIDController(50, 0, 0);

    private Translation2d speakerLocation;

    public FixAll() {
        addRequirements(TunerConstants.DriveTrain); // TODO: add other subsystems
    }

    private void configBlue() {
        speakerLocation = Constants.FieldConstants.BlueSpeakerLocation;
    }

    private void configRed() {
        speakerLocation = Constants.FieldConstants.RedSpeakerLocation;
    }

    @Override
    public void initialize() {
        robotContainer = RobotContainer.getInstance();
        oi = robotContainer.getSelectedOI();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue)
                configBlue();
            else
                configRed();
        } else {
            configBlue(); // Default to blue
        }

        driveHeading.HeadingController = headingController;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        // Heading alignment
        // let (x, y) be the difference of position between the speaker and the robot
        // swerveheading = -1*(arctan(y/x)-90deg)
        Translation2d speakerDist = speakerLocation.minus(robotContainer.robotPose.getTranslation());
        Rotation2d heading = Rotation2d.fromRadians(
                Math.atan(speakerDist.getY()/speakerDist.getX()));

        TwoDControllerInput input = oi.getXY();
        TunerConstants.DriveTrain.setControl(driveHeading
                .withTargetDirection(heading)
                .withVelocityX(-input.x() * Drive.kMaxDriveMeterS)
                .withVelocityY(-input.y() * Drive.kMaxDriveMeterS));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
