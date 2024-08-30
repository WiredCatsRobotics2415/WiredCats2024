package frc.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.generated.TunerConstants;
import frc.robot.RobotContainer;
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
            public static double arm = 70.0;
        }
        public static class field {
            public static double middle_center = 16;
            public static double middle_corner = 14.7;
            public static double top = 15;
            public static double bottom = 15;
        }

        public static class shuttle {
            public static double blue = 325.7;
            public static double red = 230.7;
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
            new WaitCommand(2),
            //new WaitUntilCommand(() -> flywheel.withinSetGoal()), 
            finger.fire(), 
            new WaitCommand(0.5),
            flywheel.off()
        ); 
    }

     public Command shootWhileMoving() {
            return new SequentialCommandGroup(
                finger.fire(), 
                new WaitCommand(0.5),
                flywheel.off()
            ); 
        }

    public Command shootSlap() {
            return new SequentialCommandGroup(
                finger.fire()
            ); 
        }
    
    public Command shootSubNoFly() {
            return new SequentialCommandGroup(
                finger.fire(), 
                new WaitCommand(0.5)
            ); 
        }

    public Command shootMiddle() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoal(Settings.field.middle_center)),
            new WaitUntilCommand(() -> arm.withinSetGoalTolerance()),
            finger.fire(), 
            new WaitCommand(0.5)
        ); 
    }

    public Command shootMiddleCorner() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoal(Settings.field.middle_corner)),
            new WaitUntilCommand(() -> arm.withinSetGoalTolerance()),
            finger.fire(), 
            new WaitCommand(0.5)
        ); 
    }

    public Command shootTop() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setGoal(Settings.field.top)),
            finger.fire(), // Change to finger.fire
            new WaitCommand(0.5)
        ); 
    }

    public Command shootBottom() {
        return new SequentialCommandGroup(
            // new InstantCommand(() -> arm.setGoal(Settings.field.bottom)),
            finger.fire(), 
            new WaitCommand(0.5)
        ); 
    }
    
    public Command shuttle() {
        if (RobotContainer.getInstance().isBlue()) {
            return TunerConstants.DriveTrain.faceAngle(Rotation2d.fromDegrees(Settings.shuttle.blue));
        }
        return TunerConstants.DriveTrain.faceAngle(Rotation2d.fromDegrees(Settings.shuttle.red));
    }
}
