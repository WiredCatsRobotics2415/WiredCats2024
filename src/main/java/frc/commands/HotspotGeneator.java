package frc.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.generated.TunerConstants;

public class HotspotGeneator {
    private static HotspotGeneator instance;
    private ArrayList<Hotspot> hotspots = new ArrayList<>(); 
    private Command currentTargetingCommand = new InstantCommand(() -> {});

    public HotspotGeneator() {
        // Load hotspots; Store in arraylist 
        hotspots.add(new Hotspot(136.5, 200)); 
        hotspots.add(new Hotspot(155, 236.5)); 
        hotspots.add(new Hotspot(136.5, 243.5)); 
        hotspots.add(new Hotspot(241.5, 295)); 
        hotspots.add(new Hotspot(241.5, 238)); 
        hotspots.add(new Hotspot(241.5, 181)); 
        hotspots.add(new Hotspot(194.5, 112.638)); 
        hotspots.add(new Hotspot(194.5, 324)); 
    }

    public static HotspotGeneator getInstance() {
        if (instance == null) {
            return new HotspotGeneator();
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
        System.out.println("tartgeting");
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
        System.out.println(currentTargetingCommand.getName());
        return currentTargetingCommand;
    }

}
