package frc.team3602.robot;

/**
 * Centralized helper for Limelight access.
 *
 * This class intentionally focuses only on the drivetrain cameras:
 * - left limelight
 * - right limelight
 *
 * Turret aiming is now pose-based, so there is no turret camera API here.
 */
public class Vision {
    public static final String LEFT_CAMERA = "limelight-left";
    public static final String RIGHT_CAMERA = "limelight-right";

    public Vision() {
        // Restrict AprilTags to the IDs your team wants to trust for localization.
        // Setting this on both cameras keeps behavior consistent between left and right.
        int[] validTagID = { 21, 26, 18, 5, 10, 2 };
        LimelightHelpers.SetFiducialIDFiltersOverride(LEFT_CAMERA, validTagID);
        LimelightHelpers.SetFiducialIDFiltersOverride(RIGHT_CAMERA, validTagID);
    }

    /**
     * Push robot heading into both Limelights.
     *
     * MegaTag2 needs a current heading to solve pose correctly. We update both
     * cameras every loop so they stay in sync.
     */
    public void setRobotOrientation(double headingDeg, double headingRateDegPerSec) {
        LimelightHelpers.SetRobotOrientation(LEFT_CAMERA, headingDeg, headingRateDegPerSec, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(RIGHT_CAMERA, headingDeg, headingRateDegPerSec, 0, 0, 0, 0);
    }

    public LimelightHelpers.PoseEstimate getLeftMegaTag2Estimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LEFT_CAMERA);
    }

    public LimelightHelpers.PoseEstimate getRightMegaTag2Estimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RIGHT_CAMERA);
    }

    /**
     * A pose estimate is considered usable when it exists and actually saw at
     * least one tag.
     */
    public boolean isPoseEstimateUsable(LimelightHelpers.PoseEstimate estimate) {
        return estimate != null && estimate.tagCount > 0;
    }
}
