package frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.LimelightResults;

/**
 * Wrapper class for LimelightHelpers in utils.
 * Continously caches a LimelightResults object and exposes methods to get Limelight info
 */
public class Vision extends SubsystemBase {
    private Pose2d cachedBackPose2d;
    private LimelightResults cachedIntakeTargetResults;

    private static Vision instance;
    private Vision() {}

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    @Override
    public void periodic() {
        cachedBackPose2d = LimelightHelpers.getBotPose2d(Constants.Vision.ShooterLimelightName);
        cachedIntakeTargetResults = LimelightHelpers.getLatestResults(Constants.Vision.IntakeLimelightName);
    }

    public Pose2d getBotPose2d() {
        return cachedBackPose2d;
    }

    public double getNoteAngleOnY() {
        return cachedIntakeTargetResults.targetingResults.targets_Detector[0].ty;
    }
}
