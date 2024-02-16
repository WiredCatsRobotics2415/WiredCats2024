package frc.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.generated.TunerConstants;
import frc.robot.Constants;

/*
 * An instance of a hotspot 
 * 
 */

public class Hotspot {
    // Target coordinates 
    private double X; 
    private double Y; 
    private Pose2d targetPose; 
    private Translation2d speakerLocation;
    private Translation2d targetCoords; 

    public Hotspot(double X, double Y) {
        this.X = X;
        this.Y = Y; 
        this.targetCoords = new Translation2d(X, Y); 
        this.targetPose = new Pose2d(this.X, this.Y, Rotation2d.fromDegrees(rotEstimation(this.X, this.Y)));

        //Do alliance preparations
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue)
                configBlue();
            else
                configRed();
        } else {
            configBlue(); // Default to blue
        }
    }

    // Set speaker location based on alliance color 
    private void configBlue() {
        speakerLocation = Constants.FieldMeasurements.BlueSpeakerLocation;
    }

    private void configRed() {
        speakerLocation = Constants.FieldMeasurements.RedSpeakerLocation;
    }

    // Calculate rotation to face speaker at hotspot 
    public double rotEstimation(double X, double Y) {
        Translation2d speakerDist = speakerLocation.minus(targetCoords);
        Rotation2d heading = Rotation2d.fromRadians(
            Math.atan2(speakerDist.getY(), speakerDist.getX())).plus(Rotation2d.fromDegrees(180));
        return heading.getDegrees(); 
    }

    public double getX() {
        return this.X; 
    }

    public double getY() {
        return this.Y; 
    }

    public Translation2d get2d() {
        return targetCoords; 
    }

    // Construct target using PathPlanner's Pathfinding program
    public Command target() {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand; 
    }



}
