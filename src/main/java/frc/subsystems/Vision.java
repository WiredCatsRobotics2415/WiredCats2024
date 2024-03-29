package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.LimelightResults;
import frc.utils.RobotPreferences;

/**
 * Wrapper class for LimelightHelpers in utils. Continously caches LimelightResults objects and
 * exposes methods to get Limelight info
 */
public class Vision extends SubsystemBase {
    private LimelightResults cachedBackTargetResults;
    private LimelightResults cachedIntakeTargetResults;

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

    @Override
    public void periodic() {
        if (Robot.isSimulation()) return;
        if (!isEnabled) return;

        // cachedBackTargetResults = LimelightHelpers.getLatestResults(Constants.Vision.ShooterLimelightName);
        cachedIntakeTargetResults = LimelightHelpers.getLatestResults(Constants.Vision.IntakeLimelightName);
    }

    /**
     * @return The targeting resuls object returned from the shooter limelight
     */
    public LimelightResults getShooterResults() {
        if (Robot.isSimulation()) {
            return new LimelightResults();
        }
        return cachedBackTargetResults;
    }

    /**
     * @return Gets the horizontal angle returned by the intake limelight's note detection.
     */
    public double getNoteAngleOnX() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getNumber("Note Detection X", 0.0d);
        }
        if (!isNoteVisible()) return 0.0d;
        return cachedIntakeTargetResults.targetingResults.targets_Detector[0].tx;
    }

    public double getNoteAngleOnY() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getNumber("Note Detection Y", 0.0d);
        }
        return cachedIntakeTargetResults.targetingResults.targets_Detector[0].ty;
    }

    public boolean isNoteVisible() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getBoolean("Note Visible", false);
        }
        return cachedIntakeTargetResults.targetingResults.targets_Detector.length != 0;
    }
}
