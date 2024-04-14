package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.OIs.OI;
import frc.utils.Logger.LogLevel;

public class DriverFeedback {
    private static XboxController currentRumbleableController = null;

    public static void flashBlackLimelight() {
        System.out.println("Telling back limelight to blink");
        LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.ShooterLimelightName);
    }

    public static void turnOffBlackLimelight() {
        System.out.println("Telling back limelight to be off");
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.ShooterLimelightName);
    }

    public static Command blinkInConfirmation() {
        return new InstantCommand(() -> flashBlackLimelight())
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> turnOffBlackLimelight()));
    }

    public static boolean controllerExists() {
        if (currentRumbleableController == null) {
            OI oi = RobotContainer.getInstance().getSelectedOI();
            if (oi == null) return false;
            currentRumbleableController = oi.getHIDOfController();
        }
        return true;
    }

    public static void rumbleSoft() {
        if (!controllerExists()) return;
        currentRumbleableController.setRumble(RumbleType.kBothRumble, Constants.DriverControl.RumbleSoftValue);
    }

    public static void noRumble() {
        if (!controllerExists()) return;
        currentRumbleableController.setRumble(RumbleType.kBothRumble, 0);
    }
}
