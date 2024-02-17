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

    public HotspotGenerator() {
        // Load hotspots; Store in arraylist 
        hotspots.add(new Hotspot(1.5, 2.1)); // Subwoofer bottom 
        hotspots.add(new Hotspot(1.9, 3.2)); // Subwoofer middle 
        hotspots.add(new Hotspot(1.5, 4.0)); // Subwoofer top 
        hotspots.add(new Hotspot(2.6, 0.96)); // Amp 
        hotspots.add(new Hotspot(2.6, 5.6)); // Furthest down 
        hotspots.add(new Hotspot(3.6, 1.7)); // Top note 
        hotspots.add(new Hotspot(3.6, 3.1)); // Middle note
        hotspots.add(new Hotspot(3.6, 4.8)); // Bottom note  
    }

    public static HotspotGenerator getInstance() {
        if (instance == null) {
            return new HotspotGenerator();
        }
        return instance; 
    } 

    public Command endCurrentCommand() {
        return new InstantCommand(() -> {
            System.out.println("ending");
            currentTargetingCommand.cancel();
        });
    }

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
