package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Owns all Limelight pose-estimation behavior.
 *
 * <p>Lifecycle:
 * <ul>
 *   <li>MegaTag 1 is used only to establish the initial field pose and gyro heading.</li>
 *   <li>MegaTag 2 is used continuously after the initial seed.</li>
 *   <li>Each camera is filtered independently, so both cameras may contribute in one loop.</li>
 * </ul>
 */
public class VisionManager {
    private static final String FRONT_CAMERA = "limelight";
    private static final String REAR_CAMERA = "limelight-rear";

    private static final String[] CAMERAS = {
        FRONT_CAMERA,
        REAR_CAMERA
    };

    // MT1 seed acceptance.
    private static final int MIN_MULTI_TAG_SEED_COUNT = 2;
    private static final double MAX_SINGLE_TAG_SEED_DISTANCE_METERS = 2.5;
    private static final double MAX_SEED_DISTANCE_METERS = 7.0;
    private static final double SEED_RETRY_SECONDS = 0.25;

    // MT2 hard rejection gates.
    private static final double MAX_MT2_ANGULAR_SPEED_RAD_PER_SEC = 3.0;
    private static final double MAX_MT2_DISTANCE_METERS = 7.0;
    private static final double MAX_SINGLE_TAG_MT2_DISTANCE_METERS = 3.5;
    private static final double MAX_VISION_TRANSLATION_ERROR_METERS = 3.0;

    // MT2 covariance model.
    private static final double BASE_XY_STD_DEV_METERS = 0.15;
    private static final double DISTANCE_STD_DEV_PER_METER = 0.15;
    private static final double SINGLE_TAG_STD_DEV_PENALTY = 0.60;
    private static final double TWO_TAG_STD_DEV_PENALTY = 0.20;
    private static final double ANGULAR_SPEED_STD_DEV_PER_RAD_PER_SEC = 0.05;
    private static final double MAX_XY_STD_DEV_METERS = 2.0;

    // MegaTag2 heading comes from the drivetrain/Pigeon, so do not trust camera yaw.
    private static final double VISION_THETA_STD_DEV_RADIANS = 9_999_999.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final boolean enabled;

    private boolean gyroSeeded = false;
    private double lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;

    public VisionManager(CommandSwerveDrivetrain drivetrain, boolean enabled) {
        this.drivetrain = drivetrain;
        this.enabled = enabled;
    }

    /** Configure values that do not need to be retransmitted every robot loop. */
    public void initialize() {
        if (!enabled) {
            return;
        }

        for (String camera : CAMERAS) {
            LimelightHelpers.SetIMUAssistAlpha(camera, 0.001);
            LimelightHelpers.setLimelightNTDouble(camera, "throttle_set", 0);
            LimelightHelpers.SetIMUMode(camera, 4);
        }

        publishSeedStatus("Vision initialized");
    }

    /**
     * Run once from Robot.robotPeriodic().
     *
     * <p>While disabled and unseeded, this periodically attempts an MT1 seed.
     * Once seeded, it continuously processes MT2 from both cameras.
     */
    public void periodic() {
        if (!enabled) {
            return;
        }

        if (!gyroSeeded) {
            if (DriverStation.isDisabled()) {
                attemptSeed(false);
            }
            return;
        }

        processMegaTag2();
    }

    /**
     * Force a fresh MT1 seed attempt. This is intentionally allowed only while disabled.
     */
    public void requestManualReseed() {
        if (!enabled) {
            publishSeedStatus("Vision disabled; reseed ignored");
            return;
        }

        if (!DriverStation.isDisabled()) {
            publishSeedStatus("Reseed rejected while enabled");
            return;
        }

        gyroSeeded = false;
        lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;
        attemptSeed(true);
    }

    /** Prepare both Limelights for disabled-time MT1 seeding. */
    public void onDisabledInit() {
        if (!enabled) {
            return;
        }

        // Keep both cameras active so either can provide the best MT1 seed.
        for (String camera : CAMERAS) {
            LimelightHelpers.setLimelightNTDouble(camera, "throttle_set", 0);
            LimelightHelpers.SetIMUMode(camera, 4);
        }

        if (!gyroSeeded) {
            lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;
            attemptSeed(true);
        }
    }

    /** Prepare both Limelights for enabled-time MegaTag2 operation. */
    public void onEnabledInit() {
        if (!enabled) {
            return;
        }

        for (String camera : CAMERAS) {
            LimelightHelpers.setLimelightNTDouble(camera, "throttle_set", 0);
            LimelightHelpers.SetIMUMode(camera, 4);
        }
    }

    public boolean isGyroSeeded() {
        return gyroSeeded;
    }

    private void attemptSeed(boolean force) {
        double now = Timer.getFPGATimestamp();
        if (!force && now - lastSeedAttemptSeconds < SEED_RETRY_SECONDS) {
            return;
        }
        lastSeedAttemptSeconds = now;

        SmartDashboard.putNumber("Vision/Seed/LastAttemptTime", now);

        SeedCandidate best = null;
        for (String camera : CAMERAS) {
            LimelightHelpers.PoseEstimate estimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);

            publishSeedCandidateTelemetry(camera, estimate);

            if (!isAcceptableSeed(estimate)) {
                continue;
            }

            SeedCandidate candidate = new SeedCandidate(camera, estimate, scoreSeed(estimate));
            if (best == null || candidate.score > best.score) {
                best = candidate;
            }
        }

        if (best == null) {
            publishSeedStatus("No acceptable MT1 seed");
            return;
        }

        seedFromPose(best.cameraName, best.estimate);
    }

    private boolean isAcceptableSeed(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null || estimate.pose == null || estimate.tagCount <= 0) {
            return false;
        }

        if (!isFinitePose(estimate.pose) || !Double.isFinite(estimate.avgTagDist)) {
            return false;
        }

        if (estimate.avgTagDist <= 0.0 || estimate.avgTagDist > MAX_SEED_DISTANCE_METERS) {
            return false;
        }

        if (estimate.tagCount >= MIN_MULTI_TAG_SEED_COUNT) {
            return true;
        }

        return estimate.tagCount == 1
                && estimate.avgTagDist <= MAX_SINGLE_TAG_SEED_DISTANCE_METERS;
    }

    private double scoreSeed(LimelightHelpers.PoseEstimate estimate) {
        // Tag count dominates; distance breaks ties and slightly favors closer solutions.
        return estimate.tagCount * 10.0 - estimate.avgTagDist;
    }

    private void seedFromPose(String cameraName, LimelightHelpers.PoseEstimate estimate) {
        Pose2d pose = estimate.pose;
        double yawDegrees = pose.getRotation().getDegrees();

        // resetPose() establishes the estimator's full field pose. Explicitly setting the
        // Pigeon keeps the physical gyro and estimator heading aligned at initialization.
        drivetrain.getPigeon2().setYaw(yawDegrees);
        drivetrain.resetPose(pose);

        gyroSeeded = true;

        SmartDashboard.putBoolean("Vision/Seed/Seeded", true);
        SmartDashboard.putString("Vision/Seed/Camera", cameraName);
        SmartDashboard.putNumber("Vision/Seed/TagCount", estimate.tagCount);
        SmartDashboard.putNumber("Vision/Seed/AverageDistanceMeters", estimate.avgTagDist);
        SmartDashboard.putNumber("Vision/Seed/YawDegrees", yawDegrees);
        SmartDashboard.putNumber("Vision/Seed/X", pose.getX());
        SmartDashboard.putNumber("Vision/Seed/Y", pose.getY());
        publishSeedStatus("Seeded from " + cameraName);
    }

    private void processMegaTag2() {
        var state = drivetrain.getState();
        double headingDegrees = state.Pose.getRotation().getDegrees();
        double omegaRadPerSec = state.Speeds.omegaRadiansPerSecond;

        for (String camera : CAMERAS) {
            processCameraMegaTag2(camera, headingDegrees, omegaRadPerSec);
        }
    }

    private void processCameraMegaTag2(
            String camera,
            double headingDegrees,
            double omegaRadPerSec) {

        LimelightHelpers.SetRobotOrientation(
                camera,
                headingDegrees,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0);

        LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);

        String rejectionReason = getMt2RejectionReason(estimate, omegaRadPerSec);
        publishMt2Telemetry(camera, estimate, omegaRadPerSec, rejectionReason);

        if (rejectionReason != null) {
            return;
        }

        double xyStdDev = computeXYStdDev(estimate, omegaRadPerSec);

        drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(
                        xyStdDev,
                        xyStdDev,
                        VISION_THETA_STD_DEV_RADIANS));

        String key = dashboardKey(camera) + "/MT2";
        SmartDashboard.putBoolean(key + "/Accepted", true);
        SmartDashboard.putString(key + "/RejectionReason", "");
        SmartDashboard.putNumber(key + "/XYStdDevMeters", xyStdDev);
    }

    /** Returns null when accepted, otherwise a human-readable rejection reason. */
    private String getMt2RejectionReason(
            LimelightHelpers.PoseEstimate estimate,
            double omegaRadPerSec) {

        if (estimate == null) {
            return "No estimate";
        }
        if (estimate.pose == null || !isFinitePose(estimate.pose)) {
            return "Invalid pose";
        }
        if (estimate.tagCount <= 0) {
            return "No tags";
        }
        if (!Double.isFinite(estimate.timestampSeconds)) {
            return "Invalid timestamp";
        }
        if (!Double.isFinite(estimate.avgTagDist) || estimate.avgTagDist <= 0.0) {
            return "Invalid distance";
        }
        if (Math.abs(omegaRadPerSec) > MAX_MT2_ANGULAR_SPEED_RAD_PER_SEC) {
            return "Angular speed too high";
        }
        if (estimate.avgTagDist > MAX_MT2_DISTANCE_METERS) {
            return "Tags too far";
        }
        if (estimate.tagCount == 1
                && estimate.avgTagDist > MAX_SINGLE_TAG_MT2_DISTANCE_METERS) {
            return "Single tag too far";
        }

        Pose2d currentPose = drivetrain.getState().Pose;
        double translationError = currentPose.getTranslation()
                .getDistance(estimate.pose.getTranslation());
        if (translationError > MAX_VISION_TRANSLATION_ERROR_METERS) {
            return "Pose jump too large";
        }

        return null;
    }

    private double computeXYStdDev(
            LimelightHelpers.PoseEstimate estimate,
            double omegaRadPerSec) {

        double stdDev = BASE_XY_STD_DEV_METERS
                + estimate.avgTagDist * DISTANCE_STD_DEV_PER_METER
                + Math.abs(omegaRadPerSec) * ANGULAR_SPEED_STD_DEV_PER_RAD_PER_SEC;

        if (estimate.tagCount == 1) {
            stdDev += SINGLE_TAG_STD_DEV_PENALTY;
        } else if (estimate.tagCount == 2) {
            stdDev += TWO_TAG_STD_DEV_PENALTY;
        }

        return Math.min(stdDev, MAX_XY_STD_DEV_METERS);
    }

    private boolean isFinitePose(Pose2d pose) {
        return Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private void publishSeedCandidateTelemetry(
            String camera,
            LimelightHelpers.PoseEstimate estimate) {
        String key = dashboardKey(camera) + "/MT1";
        SmartDashboard.putBoolean(key + "/Valid", isAcceptableSeed(estimate));
        SmartDashboard.putNumber(key + "/TagCount", estimate != null ? estimate.tagCount : 0);
        SmartDashboard.putNumber(
                key + "/AverageDistanceMeters",
                estimate != null ? estimate.avgTagDist : -1.0);
        SmartDashboard.putNumber(
                key + "/Score",
                isAcceptableSeed(estimate) ? scoreSeed(estimate) : -1.0);
    }

    private void publishMt2Telemetry(
            String camera,
            LimelightHelpers.PoseEstimate estimate,
            double omegaRadPerSec,
            String rejectionReason) {
        String key = dashboardKey(camera) + "/MT2";

        SmartDashboard.putBoolean(key + "/Accepted", rejectionReason == null);
        SmartDashboard.putString(
                key + "/RejectionReason",
                rejectionReason == null ? "" : rejectionReason);
        SmartDashboard.putNumber(key + "/AngularSpeedRadPerSec", omegaRadPerSec);
        SmartDashboard.putNumber(key + "/TagCount", estimate != null ? estimate.tagCount : 0);
        SmartDashboard.putNumber(
                key + "/AverageDistanceMeters",
                estimate != null ? estimate.avgTagDist : -1.0);

        if (estimate != null && estimate.pose != null) {
            SmartDashboard.putNumber(key + "/X", estimate.pose.getX());
            SmartDashboard.putNumber(key + "/Y", estimate.pose.getY());
            SmartDashboard.putNumber(
                    key + "/TranslationErrorMeters",
                    drivetrain.getState().Pose.getTranslation()
                            .getDistance(estimate.pose.getTranslation()));
        }
    }

    private String dashboardKey(String camera) {
        return "Vision/" + camera;
    }

    private void publishSeedStatus(String status) {
        SmartDashboard.putString("Vision/Seed/Status", status);
        SmartDashboard.putBoolean("Vision/Seed/Seeded", gyroSeeded);
    }

    private static final class SeedCandidate {
        private final String cameraName;
        private final LimelightHelpers.PoseEstimate estimate;
        private final double score;

        private SeedCandidate(
                String cameraName,
                LimelightHelpers.PoseEstimate estimate,
                double score) {
            this.cameraName = cameraName;
            this.estimate = estimate;
            this.score = score;
        }
    }
}
