package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class DriverFeedback {
    public static void flashBlackLimelight() {
        LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.ShooterLimelightName);
    }

    public static void turnOffBlackLimelight() {
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.ShooterLimelightName);
    }

    public static Command blinkInConfirmation() {
        return new InstantCommand(() -> flashBlackLimelight())
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> turnOffBlackLimelight()));
    }
}
