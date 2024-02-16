package frc.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.generated.TunerConstants;

public class HotspotGeneator {
    private static HotspotGeneator instance;
    private ArrayList<Hotspot> hotspots = new ArrayList<>(); 

    public HotspotGeneator() {
        // Load hotspots; Store in arraylist 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
        hotspots.add(new Hotspot(0, 0)); 
    }

    public static HotspotGeneator getInstance() {
        if (instance == null) {
            return new HotspotGeneator();
        }
        return instance; 
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

        return closestHotspot.target(); 
    }

}
