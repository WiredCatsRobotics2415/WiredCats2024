package frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.utils.LimelightHelpers.LimelightResults;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.LimelightHelpers.RawFiducial;
import frc.utils.DriverFeedback;
import frc.utils.LimelightHelpers;
import frc.utils.RobotPreferences;

/**
 * Wrapper class for LimelightHelpers in utils. Continously caches LimelightResults objects and
 * exposes methods to get Limelight info
 */
public class Vision extends SubsystemBase {
    private static Vision instance;
    private boolean isEnabled = false;

    private Vision() {
        if (Robot.isSimulation()) {
            SmartDashboard.setDefaultNumber("Note detection X", 0.0d);
            SmartDashboard.setDefaultBoolean("Note Visible", false);
        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    /**
     * Enables or disables this subsystem. If enabled, and a limelight is not present, then there
     * will be a constant log of errors.
     * IF IN SIMULATION, it will automatically be disabled
     */
    public void setPreferences() {
        isEnabled = RobotPreferences.shouldUseLimelight();
        if (Robot.isSimulation()) isEnabled = false;
    }

    /**
     * @return The targeting resuls object returned from the shooter limelight
     */
    public PoseEstimate getShooterResults() {
        if (Robot.isSimulation()) {
            return new PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0, new RawFiducial[0]);
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.ShooterLimelightName);
    }

    /**
     * @return Gets the horizontal angle returned by the intake limelight's note detection.
     */
    public double getNoteAngleOnX() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getNumber("Note detection X", 0.0d);
        }
        return LimelightHelpers.getTX(Constants.Vision.IntakeLimelightName);
    }

    public boolean isNoteVisible() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getBoolean("Note Visible", false);
        }
        return LimelightHelpers.getTY(Constants.Vision.IntakeLimelightName) != 0;
    }

    @Override
    public void periodic() {
        if (isNoteVisible()) DriverFeedback.rumbleSoft();
        else DriverFeedback.noRumble();
    }
}
