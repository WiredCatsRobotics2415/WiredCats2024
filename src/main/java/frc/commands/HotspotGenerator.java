package frc.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.generated.TunerConstants;

public class HotspotGenerator {
    private static HotspotGenerator instance;
    private ArrayList<Hotspot> hotspots = new ArrayList<>(); 
    private Command currentTargetingCommand;

    /**
     * Constructs all Hotspots with thier appropriate positions,
     * according to hotspots.webp.
     */
    public HotspotGenerator() {
        hotspots.add(new Hotspot(0.66, 4.39)); // Subwoofer bottom 
        hotspots.add(new Hotspot(1.3, 5.54)); // Subwoofer middle 
        hotspots.add(new Hotspot(0.74, 6.64)); // Subwoofer top 
        /* 
        hotspots.add(new Hotspot(2.6, 0.96)); // Amp 
        hotspots.add(new Hotspot(2.6, 5.6)); // Furthest down 
        hotspots.add(new Hotspot(3.6, 1.7)); // Top note 
        hotspots.add(new Hotspot(3.6, 3.1)); // Middle note
        hotspots.add(new Hotspot(3.6, 4.8)); // Bottom note  
        */ 
    }

    public static HotspotGenerator getInstance() {
        if (instance == null) {
            return new HotspotGenerator();
        }
        return instance; 
    } 

    /**
     * @return a Command that, when run, cancels the current targeting command.
     */
    public Command endCurrentCommand() {
        return new InstantCommand(() -> {
            System.out.println("ending");
            currentTargetingCommand.cancel();
        });
    }

    /**
     * @return a Command that is intended to run on SwerveDrive to go to the nearest hotspot,
     * based on the robot's current pose.
     */
    public Command targetClosest() {
        Translation2d current_pose = TunerConstants.DriveTrain.getRobotPose().getTranslation(); 
        Hotspot closestHotspot = hotspots.get(0); 
        double minDistance = closestHotspot.get2d().getDistance(current_pose); 
        for (Hotspot hotspot : hotspots) {
            double distance = current_pose.getDistance(hotspot.get2d());
            if (distance < minDistance) {
                closestHotspot = hotspot; 
                minDistance = distance; 
            }
        }

        currentTargetingCommand = closestHotspot.target(); 
        System.out.println(currentTargetingCommand);
        return currentTargetingCommand; 
    }

}
