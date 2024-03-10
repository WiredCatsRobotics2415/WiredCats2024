package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.OIs.OI;
import frc.robot.OIs.OI.TwoDControllerInput;
import frc.robot.RobotContainer;

public class TargetHotspot extends Command {
    //GENERAL
    private Command currentPathfindingCommand;

    public TargetHotspot() {
        addRequirements(TunerConstants.DriveTrain); // TODO: add other subsystems
    }

    @Override
    public void initialize() {
        System.out.println("tartgeting");
        Translation2d currentPose = TunerConstants.DriveTrain.getRobotPose().getTranslation(); 
        //System.out.println(currentPose);
        Hotspot closestHotspot = Constants.Hotspots.get(0); 
        double minDistance = closestHotspot.get2d().getDistance(currentPose); 
        for (Hotspot hotspot : Constants.Hotspots) {
            double distance = currentPose.getDistance(hotspot.get2d());
            if (distance < minDistance) {
                closestHotspot = hotspot; 
                minDistance = distance; 
            }
        }
        // System.out.println(closestHotspot.get2d());
        // currentPose = TunerConstants.DriveTrain.getRobotPose().getTranslation(); 
        currentPathfindingCommand = closestHotspot.target();
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(currentPathfindingCommand);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("done pathfinding");
        currentPathfindingCommand.end(true);
    }

    @Override
    public boolean isFinished() {
        return currentPathfindingCommand.isFinished();
    }
}
