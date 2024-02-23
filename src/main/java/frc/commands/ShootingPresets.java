package frc.commands;

import frc.subsystems.Arm;
import frc.subsystems.NewFlywheel;

public class ShootingPresets {
    // declare shooting-related subsystems 
    private Arm arm; 
    private NewFlywheel flywheel; 
    public static ShootingPresets instance; 

    public ShootingPresets() {
        arm = Arm.getInstance(); 
        flywheel = NewFlywheel.getInstance(); 
    }

    public static ShootingPresets getInstance() {
        if (instance == null) {
            instance = new ShootingPresets();
        }
        return instance; 
    }

    // record the angle and flywheel speeds for hotspots 
    public static class Settings {
        public static class subwoofer {
            public static double arm = 0.0; 
            public static double left_flywheel = 0.0; 
            public static double right_flywheel = 0.0; 
        }
    }

    public void subwoofer() {
        arm.setGoal(Settings.subwoofer.arm);
        flywheel.on(Settings.subwoofer.left_flywheel, Settings.subwoofer.right_flywheel);  
    }
}
