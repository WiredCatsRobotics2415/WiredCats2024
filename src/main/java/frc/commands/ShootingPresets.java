package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.subsystems.Arm;
import frc.subsystems.Flywheel;
import frc.subsystems.Finger; 
import frc.subsystems.Intake;

public class ShootingPresets {
    // declare shooting-related subsystems 
    private Arm arm; 
    private Flywheel flywheel; 
    private Finger finger;
    public static ShootingPresets instance; 
    private Intake intake;

    public ShootingPresets() {
        arm = Arm.getInstance(); 
        flywheel = Flywheel.getInstance(); 
        finger = Finger.getInstance();
        intake = Intake.getInstance();
    }

    public static ShootingPresets getInstance() {
        if (instance == null) {
            instance = new ShootingPresets();
        }
        return instance; 
    }

    // Angle and flywheel speeds at each location. 
    public static class Settings {
        public static class subwoofer {
            public static double arm = 0.0; 
            public static double left_flywheel = 6000; 
            public static double right_flywheel = 8000; 
        }

        public static class amp {
            public static double arm = 80.0;
        }
    }

    // Fire next to subwoofer.  
    public Command shootClose() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> arm.setGoal(Settings.subwoofer.arm)),
            flywheel.on(Settings.subwoofer.left_flywheel, Settings.subwoofer.right_flywheel)); 
    }

    // Fire next to amp. 
    public Command shootAmp() {
        return new InstantCommand(() -> 
            arm.setGoal(Settings.amp.arm) 
        );
    }

    // Fire next to subwoofer and then stop the flywheel during autonomous.
    public Command subwooferAuto() {
        return new SequentialCommandGroup(
            shootClose(),  
            new WaitCommand(3),
            //new WaitUntilCommand(() -> flywheel.withinSetGoal()), 
            finger.fire(), 
            intake.intakeNote(),
            new WaitCommand(2),
            flywheel.off()
        ); 
    }
}
