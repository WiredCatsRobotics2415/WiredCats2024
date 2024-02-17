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
import frc.robot.Constants;

/*
 * An instance of a hotspot 
 * 
 */

public class Hotspot {
    // Target coordinates 
    private double x; 
    private double y; 
    private Pose2d targetPose; 
    private Translation2d speakerLocation = Constants.FieldMeasurements.BlueSpeakerLocation;
    private Translation2d targetCoords;

    public Hotspot(double x, double y) {
        this.x = x;
        this.y = y; 
        this.targetCoords = new Translation2d(this.x, this.y); 
        this.targetPose = new Pose2d(this.x, this.y, Rotation2d.fromDegrees(rotEstimation(this.x, this.y)));

        //Do alliance preparations
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) configBlue();
            else configRed();
        } else configBlue();
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
        return this.x; 
    }

    public double getY() {
        return this.y; 
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
