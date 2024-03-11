package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.subsystems.Arm;
import frc.subsystems.Flywheel;

public class ShootingPresets {
    // declare shooting-related subsystems 
    private Arm arm; 
    private Flywheel flywheel; 
    public static ShootingPresets instance; 

    public ShootingPresets() {
        arm = Arm.getInstance(); 
        flywheel = Flywheel.getInstance(); 
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
            public static double left_flywheel = 6000; 
            public static double right_flywheel = 8000; 
        }
    }

    // fire next to subwoofer 
    public Command shootClose() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> arm.setGoal(Settings.subwoofer.arm)),
            new InstantCommand(() -> flywheel.on(Settings.subwoofer.left_flywheel, Settings.subwoofer.right_flywheel))); 
    }

    public void subwoofer() {
        arm.setGoal(Settings.subwoofer.arm);
        flywheel.on(Settings.subwoofer.left_flywheel, Settings.subwoofer.right_flywheel).schedule();  
    }
}
