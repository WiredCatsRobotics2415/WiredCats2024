package frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.Robot;
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
    private SwerveDrive swerveInstance;

    private Vision() {
        if (Robot.isSimulation()) {
            SmartDashboard.setDefaultNumber("Note detection Y", 0.0d);
            swerveInstance = TunerConstants.DriveTrain;
        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) return;
        cachedBackPose2d = LimelightHelpers.getBotPose2d(Constants.Vision.ShooterLimelightName);
        cachedIntakeTargetResults = LimelightHelpers.getLatestResults(Constants.Vision.IntakeLimelightName);
    }

    public Pose2d getBotPose2d() {
        if (Robot.isSimulation()) {
            return swerveInstance.getRobotPose();
        }
        return cachedBackPose2d;
    }

    public double getNoteAngleOnY() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getNumber("Note Detection Y", 0.0d);
        }
        return cachedIntakeTargetResults.targetingResults.targets_Detector[0].ty;
    }
}