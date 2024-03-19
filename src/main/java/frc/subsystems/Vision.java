package frc.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.LimelightResults;
import frc.utils.RobotPreferences;

/**
 * Wrapper class for LimelightHelpers in utils. Continously caches a LimelightResults object and
 * exposes methods to get Limelight info
 */
public class Vision extends SubsystemBase {
    private Pose2d cachedBackPose2d;
    private boolean backPoseCouldSeeTarget = false;
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
     */
    public void setPreferences() {
        isEnabled = RobotPreferences.shouldUseLimelight();
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) return;
        if (!isEnabled) return;
        cachedBackPose2d =
                LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.ShooterLimelightName);
        backPoseCouldSeeTarget = LimelightHelpers.getTV(Constants.Vision.ShooterLimelightName);

        // TODO: is the following line necessary? added at walton
        // cachedBackPose2d = new Pose2d(-cachedBackPose2d.getX(), -cachedBackPose2d.getY(),
        // cachedBackPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        // cachedIntakeTargetResults =
        // LimelightHelpers.getLatestResults(Constants.Vision.IntakeLimelightName);
    }

    /**
     * @return The Pose2D returned from the shooter limelight's MegaTag measurements.
     */
    public Pose2d getBotPose2d() {
        if (Robot.isSimulation()) {
            return TunerConstants.DriveTrain.getRobotPose();
        }
        return cachedBackPose2d;
    }

    /**
     * @return If the last vision measuremnt from the shooter limelight actually used an AprilTag
     */
    public boolean couldSeeApriltag() {
        return backPoseCouldSeeTarget;
    }

    /**
     * @return Gets the horizontal angle returned by the intake limelight's note detection.
     */
    public double getNoteAngleOnX() {
        if (Robot.isSimulation()) {
            return SmartDashboard.getNumber("Note Detection X", 0.0d);
        }
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
        } else if (cachedIntakeTargetResults.targetingResults.targets_Detector[0].ta > 0) {
            return true;
        } else {
            return false;
        }
    }
}
