package frc.robot;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Owns all Limelight pose-estimation behavior.
 *
 * <p>
 * Lifecycle:
 * <ul>
 * <li>MegaTag 1 continuously establishes/refines field pose while
 * disabled.</li>
 * <li>MegaTag 2 is used continuously while enabled after at least one valid
 * seed.</li>
 * <li>Each camera is represented by a VisionCamera object and filtered
 * independently.</li>
 * <li>Both accepted cameras may contribute measurements during the same robot
 * loop.</li>
 * <li>MT2 covariance is deterministic and based only on tag count and
 * distance.</li>
 * </ul>
 */
public class VisionManager {
    private static final double MAX_MEASUREMENT_AGE_SECONDS = 0.25;
    private static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.05;

    // MT1 seed rules.
    private static final int MIN_MULTI_TAG_SEED_COUNT = 2;
    private static final double MAX_SINGLE_TAG_SEED_DISTANCE_METERS = 2.5;
    private static final double MAX_SEED_DISTANCE_METERS = 7.0;
    private static final double SEED_RETRY_SECONDS = 0.25;

    // MT2 hard rejection rules.
    private static final double MAX_MT2_ANGULAR_SPEED_RAD_PER_SEC = 3.0;
    private static final double MAX_MT2_DISTANCE_METERS = 7.0;
    private static final double MAX_SINGLE_TAG_MT2_DISTANCE_METERS = 3.5;
    private static final double MAX_VISION_TRANSLATION_ERROR_METERS = 3.0;

    // Deterministic covariance model.
    private static final double ONE_TAG_BASE_STD_DEV_METERS = 1.20;
    private static final double TWO_TAG_BASE_STD_DEV_METERS = 0.60;
    private static final double THREE_TAG_BASE_STD_DEV_METERS = 0.35;
    private static final double FOUR_PLUS_TAG_BASE_STD_DEV_METERS = 0.20;
    private static final double DISTANCE_STD_DEV_PER_METER = 0.10;
    private static final double MAX_XY_STD_DEV_METERS = 2.0;

    // MegaTag2 heading comes from the drivetrain/Pigeon, so do not trust camera
    // yaw.
    private static final double VISION_THETA_STD_DEV_RADIANS = 9_999_999.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final boolean enabled;
    private final List<VisionCamera> cameras;

    private boolean gyroSeeded = false;
    private double lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;

    private GenericEntry seededEntry;
    private GenericEntry seedStatusEntry;
    private GenericEntry seedCameraEntry;
    private GenericEntry seedTagCountEntry;
    private GenericEntry seedDistanceEntry;
    private GenericEntry seedAgeEntry;
    private GenericEntry seedXEntry;
    private GenericEntry seedYEntry;
    private GenericEntry seedYawEntry;

    public VisionManager(CommandSwerveDrivetrain drivetrain, boolean enabled) {
        this.drivetrain = drivetrain;
        this.enabled = enabled;
        this.cameras = List.of(
                new VisionCamera("limelight", "Front"),
                new VisionCamera("limelight-rear", "Rear"));
    }

    public void initialize() {
        initializeShuffleboard();

        if (!enabled) {
            publishSeedStatus("Vision disabled");
            return;
        }

        for (VisionCamera camera : cameras) {
            LimelightHelpers.SetIMUAssistAlpha(camera.name, 0.001);
            configureCameraForPoseEstimation(camera);
        }

        publishSeedStatus("Vision initialized");
    }

    public void periodic() {
        if (!enabled) {
            return;
        }

        if (DriverStation.isDisabled()) {
            if (!gyroSeeded) {
                attemptSeed(false);
            } else {
                // Continue refining X/Y using MT2 while disabled.
                processMegaTag2();
            }
            return;
        }

        if (!gyroSeeded) {
            publishSeedStatus("Enabled without a valid MT1 seed");
            return;
        }

        processMegaTag2();
    }

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
        if (!gyroSeeded) {
            lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;
            attemptSeed(true);
        }
    }

    public void onDisabledInit() {
        if (!enabled) {
            return;
        }

        for (VisionCamera camera : cameras) {
            configureCameraForPoseEstimation(camera);
        }

        if (!gyroSeeded) {
            lastSeedAttemptSeconds = Double.NEGATIVE_INFINITY;
            attemptSeed(true);
        }
    }

    public void onEnabledInit() {
        if (!enabled) {
            return;
        }

        for (VisionCamera camera : cameras) {
            configureCameraForPoseEstimation(camera);
        }
    }

    public boolean isGyroSeeded() {
        return gyroSeeded;
    }

    private void configureCameraForPoseEstimation(VisionCamera camera) {
        LimelightHelpers.setLimelightNTDouble(camera.name, "throttle_set", 0);
        LimelightHelpers.SetIMUMode(camera.name, 4);
    }

    private void attemptSeed(boolean force) {
        double now = Timer.getFPGATimestamp();

        if (!force && now - lastSeedAttemptSeconds < SEED_RETRY_SECONDS) {
            return;
        }

        lastSeedAttemptSeconds = now;
        SeedCandidate best = null;

        for (VisionCamera camera : cameras) {
            LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera.name);

            camera.lastMt1Estimate = estimate;
            camera.lastMt1AgeSeconds = measurementAgeSeconds(estimate, now);

            String rejectionReason = getMt1RejectionReason(estimate, now);
            camera.lastMt1Accepted = rejectionReason == null;
            camera.lastMt1RejectionReason = rejectionReason == null ? "" : rejectionReason;

            publishSeedCandidateTelemetry(camera);

            if (!camera.lastMt1Accepted) {
                continue;
            }

            SeedCandidate candidate = new SeedCandidate(camera, estimate);

            if (best == null || isBetterSeed(candidate, best)) {
                best = candidate;
            }
        }

        if (best == null) {
            publishSeedStatus("No acceptable MT1 seed");
            return;
        }

        seedFromPose(best);
    }

    private boolean isBetterSeed(
            SeedCandidate candidate,
            SeedCandidate currentBest) {

        if (candidate.estimate.tagCount != currentBest.estimate.tagCount) {
            return candidate.estimate.tagCount > currentBest.estimate.tagCount;
        }

        if (candidate.estimate.avgTagDist != currentBest.estimate.avgTagDist) {
            return candidate.estimate.avgTagDist < currentBest.estimate.avgTagDist;
        }

        return candidate.camera.name
                .compareTo(currentBest.camera.name) < 0;
    }

    private String getMt1RejectionReason(
            LimelightHelpers.PoseEstimate estimate,
            double nowSeconds) {

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

        double ageSeconds = measurementAgeSeconds(estimate, nowSeconds);

        if (ageSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
            return "Stale measurement";
        }

        if (ageSeconds < -MAX_FUTURE_TIMESTAMP_SECONDS) {
            return "Timestamp in future";
        }

        if (!Double.isFinite(estimate.avgTagDist)
                || estimate.avgTagDist <= 0.0) {
            return "Invalid distance";
        }

        if (estimate.avgTagDist > MAX_SEED_DISTANCE_METERS) {
            return "Tags too far";
        }

        if (estimate.tagCount == 1
                && estimate.avgTagDist > MAX_SINGLE_TAG_SEED_DISTANCE_METERS) {
            return "Single tag too far";
        }

        return null;
    }

    private void seedFromPose(SeedCandidate candidate) {
        Pose2d pose = candidate.estimate.pose;
        double yawDegrees = pose.getRotation().getDegrees();

        drivetrain.getPigeon2().setYaw(yawDegrees);
        drivetrain.resetPose(pose);

        gyroSeeded = true;

        publishSeedDetails(candidate, yawDegrees);
        publishSeedStatus("Seeded from " + candidate.camera.displayName);
    }

    private void processMegaTag2() {
        var state = drivetrain.getState();
        double headingDegrees = state.Pose.getRotation().getDegrees();
        double omegaRadPerSec = state.Speeds.omegaRadiansPerSecond;
        double now = Timer.getFPGATimestamp();

        for (VisionCamera camera : cameras) {
            processCameraMegaTag2(
                    camera,
                    headingDegrees,
                    omegaRadPerSec,
                    now);
        }
    }

    private void processCameraMegaTag2(
            VisionCamera camera,
            double headingDegrees,
            double omegaRadPerSec,
            double nowSeconds) {

        LimelightHelpers.SetRobotOrientation(
                camera.name,
                headingDegrees,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0);

        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                camera.name);

        camera.lastMt2Estimate = estimate;
        camera.lastMt2AgeSeconds = measurementAgeSeconds(estimate, nowSeconds);

        String rejectionReason = getMt2RejectionReason(
                estimate,
                omegaRadPerSec,
                nowSeconds);

        camera.lastMt2Accepted = rejectionReason == null;
        camera.lastMt2RejectionReason = rejectionReason == null ? "" : rejectionReason;

        if (!camera.lastMt2Accepted) {
            camera.lastMt2StdDevMeters = MAX_XY_STD_DEV_METERS;
            publishMt2Telemetry(camera, omegaRadPerSec);
            return;
        }

        double xyStdDev = computeXYStdDev(estimate);
        camera.lastMt2StdDevMeters = xyStdDev;

        drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(
                        xyStdDev,
                        xyStdDev,
                        VISION_THETA_STD_DEV_RADIANS));

        camera.lastMt2AcceptedFpgaTime = nowSeconds;
        publishMt2Telemetry(camera, omegaRadPerSec);
    }

    private String getMt2RejectionReason(
            LimelightHelpers.PoseEstimate estimate,
            double omegaRadPerSec,
            double nowSeconds) {

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

        double ageSeconds = measurementAgeSeconds(estimate, nowSeconds);

        if (ageSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
            return "Stale measurement";
        }

        if (ageSeconds < -MAX_FUTURE_TIMESTAMP_SECONDS) {
            return "Timestamp in future";
        }

        if (!Double.isFinite(estimate.avgTagDist)
                || estimate.avgTagDist <= 0.0) {
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
            LimelightHelpers.PoseEstimate estimate) {

        double stdDev;

        if (estimate.tagCount <= 1) {
            stdDev = ONE_TAG_BASE_STD_DEV_METERS;
        } else if (estimate.tagCount == 2) {
            stdDev = TWO_TAG_BASE_STD_DEV_METERS;
        } else if (estimate.tagCount == 3) {
            stdDev = THREE_TAG_BASE_STD_DEV_METERS;
        } else {
            stdDev = FOUR_PLUS_TAG_BASE_STD_DEV_METERS;
        }

        stdDev += estimate.avgTagDist
                * DISTANCE_STD_DEV_PER_METER;

        return Math.min(stdDev, MAX_XY_STD_DEV_METERS);
    }

    private double measurementAgeSeconds(
            LimelightHelpers.PoseEstimate estimate,
            double nowSeconds) {

        if (estimate == null
                || !Double.isFinite(estimate.timestampSeconds)) {
            return Double.POSITIVE_INFINITY;
        }

        return nowSeconds - estimate.timestampSeconds;
    }

    private boolean isFinitePose(Pose2d pose) {
        return Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private void initializeShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        seededEntry = tab.add("Gyro Seeded", false)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

        seedStatusEntry = tab.add("Seed Status", "Not initialized")
                .withPosition(2, 0)
                .withSize(4, 1)
                .getEntry();

        seedCameraEntry = tab.add("Seed Camera", "")
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

        seedTagCountEntry = tab.add("Seed Tags", 0.0)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();

        seedDistanceEntry = tab.add("Seed Distance m", -1.0)
                .withPosition(3, 1)
                .withSize(2, 1)
                .getEntry();

        seedAgeEntry = tab.add("Seed Age s", -1.0)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

        seedXEntry = tab.add("Seed X", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        seedYEntry = tab.add("Seed Y", 0.0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();

        seedYawEntry = tab.add("Seed Yaw", 0.0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        for (int index = 0; index < cameras.size(); index++) {
            cameras.get(index).initializeShuffleboard(tab, index);
        }
    }

    private void publishSeedCandidateTelemetry(VisionCamera camera) {
        LimelightHelpers.PoseEstimate estimate = camera.lastMt1Estimate;

        camera.mt1AcceptedEntry.setBoolean(camera.lastMt1Accepted);
        camera.mt1ReasonEntry.setString(camera.lastMt1RejectionReason);
        camera.mt1AgeEntry.setDouble(camera.lastMt1AgeSeconds);
        camera.mt1TagCountEntry.setDouble(
                estimate != null ? estimate.tagCount : 0);
        camera.mt1DistanceEntry.setDouble(
                estimate != null ? estimate.avgTagDist : -1.0);

    // Also publish mirror keys for non-Shuffleboard NT clients (Elastic/AdvantageScope)
    String prefix = "Vision/" + camera.displayName + "/MT1/";
    SmartDashboard.putBoolean(prefix + "Accepted", camera.lastMt1Accepted);
    SmartDashboard.putString(prefix + "Reason", camera.lastMt1RejectionReason);
    SmartDashboard.putNumber(prefix + "Age", camera.lastMt1AgeSeconds);
    SmartDashboard.putNumber(prefix + "Tags", estimate != null ? estimate.tagCount : 0);
    SmartDashboard.putNumber(prefix + "Distance", estimate != null ? estimate.avgTagDist : -1.0);
    }

    private void publishMt2Telemetry(
            VisionCamera camera,
            double omegaRadPerSec) {

        LimelightHelpers.PoseEstimate estimate = camera.lastMt2Estimate;

        camera.mt2AcceptedEntry.setBoolean(camera.lastMt2Accepted);
        camera.mt2ReasonEntry.setString(
                camera.lastMt2RejectionReason);
        camera.mt2AgeEntry.setDouble(camera.lastMt2AgeSeconds);
        camera.mt2StdDevEntry.setDouble(
                camera.lastMt2StdDevMeters);
        camera.mt2OmegaEntry.setDouble(omegaRadPerSec);
        camera.mt2TagCountEntry.setDouble(
                estimate != null ? estimate.tagCount : 0);
        camera.mt2DistanceEntry.setDouble(
                estimate != null ? estimate.avgTagDist : -1.0);

        if (estimate != null && estimate.pose != null) {
            camera.mt2XEntry.setDouble(estimate.pose.getX());
            camera.mt2YEntry.setDouble(estimate.pose.getY());
            camera.mt2TranslationErrorEntry.setDouble(
                    drivetrain.getState().Pose.getTranslation()
                            .getDistance(
                                    estimate.pose.getTranslation()));
        } else {
            camera.mt2XEntry.setDouble(0.0);
            camera.mt2YEntry.setDouble(0.0);
            camera.mt2TranslationErrorEntry.setDouble(-1.0);
        }

        camera.mt2LastAcceptedTimeEntry.setDouble(
                camera.lastMt2AcceptedFpgaTime);

    // Mirror MT2 telemetry for NetworkTables clients that don't use Shuffleboard layout
    String prefix = "Vision/" + camera.displayName + "/MT2/";
    SmartDashboard.putBoolean(prefix + "Accepted", camera.lastMt2Accepted);
    SmartDashboard.putString(prefix + "Reason", camera.lastMt2RejectionReason);
    SmartDashboard.putNumber(prefix + "Age", camera.lastMt2AgeSeconds);
    SmartDashboard.putNumber(prefix + "XYStdDev", camera.lastMt2StdDevMeters);
    SmartDashboard.putNumber(prefix + "Omega", omegaRadPerSec);
    SmartDashboard.putNumber(prefix + "Tags", estimate != null ? estimate.tagCount : 0);
    SmartDashboard.putNumber(prefix + "Distance", estimate != null ? estimate.avgTagDist : -1.0);
    SmartDashboard.putNumber(prefix + "LastAcceptedTime", camera.lastMt2AcceptedFpgaTime);
    SmartDashboard.putNumber(prefix + "X", estimate != null && estimate.pose != null ? estimate.pose.getX() : 0.0);
    SmartDashboard.putNumber(prefix + "Y", estimate != null && estimate.pose != null ? estimate.pose.getY() : 0.0);
    SmartDashboard.putNumber(prefix + "TranslationError", estimate != null && estimate.pose != null ?
        drivetrain.getState().Pose.getTranslation().getDistance(estimate.pose.getTranslation()) : -1.0);
    }

    private void publishSeedDetails(
            SeedCandidate candidate,
            double yawDegrees) {

        LimelightHelpers.PoseEstimate estimate = candidate.estimate;

        seedCameraEntry.setString(candidate.camera.displayName);
        seedTagCountEntry.setDouble(estimate.tagCount);
        seedDistanceEntry.setDouble(estimate.avgTagDist);
        seedAgeEntry.setDouble(
                candidate.camera.lastMt1AgeSeconds);
        seedXEntry.setDouble(estimate.pose.getX());
        seedYEntry.setDouble(estimate.pose.getY());
        seedYawEntry.setDouble(yawDegrees);

        SmartDashboard.putString(
                "Vision/Seed/Camera",
                candidate.camera.name);
    }

    private void publishSeedStatus(String status) {
        if (seededEntry != null) {
            seededEntry.setBoolean(gyroSeeded);
            seedStatusEntry.setString(status);
        }

        SmartDashboard.putString(
                "Vision/Seed/Status",
                status);
        SmartDashboard.putBoolean(
                "Vision/Seed/Seeded",
                gyroSeeded);
    }

    private static final class VisionCamera {
        private final String name;
        private final String displayName;

        private LimelightHelpers.PoseEstimate lastMt1Estimate;
        private boolean lastMt1Accepted;
        private String lastMt1RejectionReason = "Not processed";
        private double lastMt1AgeSeconds = Double.POSITIVE_INFINITY;

        private LimelightHelpers.PoseEstimate lastMt2Estimate;
        private boolean lastMt2Accepted;
        private String lastMt2RejectionReason = "Not processed";
        private double lastMt2AgeSeconds = Double.POSITIVE_INFINITY;
        private double lastMt2StdDevMeters = MAX_XY_STD_DEV_METERS;
        private double lastMt2AcceptedFpgaTime = -1.0;

        private GenericEntry mt1AcceptedEntry;
        private GenericEntry mt1ReasonEntry;
        private GenericEntry mt1AgeEntry;
        private GenericEntry mt1TagCountEntry;
        private GenericEntry mt1DistanceEntry;

        private GenericEntry mt2AcceptedEntry;
        private GenericEntry mt2ReasonEntry;
        private GenericEntry mt2AgeEntry;
        private GenericEntry mt2StdDevEntry;
        private GenericEntry mt2OmegaEntry;
        private GenericEntry mt2TagCountEntry;
        private GenericEntry mt2DistanceEntry;
        private GenericEntry mt2XEntry;
        private GenericEntry mt2YEntry;
        private GenericEntry mt2TranslationErrorEntry;
        private GenericEntry mt2LastAcceptedTimeEntry;

        private VisionCamera(
                String name,
                String displayName) {
            this.name = name;
            this.displayName = displayName;
        }

        private void initializeShuffleboard(
                ShuffleboardTab tab,
                int cameraIndex) {

            int baseColumn = cameraIndex * 6;
            int mt1Row = 4;
            int mt2Row = 7;

            tab.add(displayName + " Camera", name)
                    .withPosition(baseColumn, 3)
                    .withSize(6, 1);

            mt1AcceptedEntry = tab.add(
                    displayName + " MT1 Accepted",
                    false)
                    .withPosition(baseColumn, mt1Row)
                    .withSize(2, 1)
                    .getEntry();

            mt1ReasonEntry = tab.add(
                    displayName + " MT1 Reason",
                    "Not processed")
                    .withPosition(baseColumn + 2, mt1Row)
                    .withSize(4, 1)
                    .getEntry();

            mt1TagCountEntry = tab.add(
                    displayName + " MT1 Tags",
                    0.0)
                    .withPosition(baseColumn, mt1Row + 1)
                    .withSize(1, 1)
                    .getEntry();

            mt1DistanceEntry = tab.add(
                    displayName + " MT1 Distance m",
                    -1.0)
                    .withPosition(baseColumn + 1, mt1Row + 1)
                    .withSize(2, 1)
                    .getEntry();

            mt1AgeEntry = tab.add(
                    displayName + " MT1 Age s",
                    -1.0)
                    .withPosition(baseColumn + 3, mt1Row + 1)
                    .withSize(2, 1)
                    .getEntry();

            mt2AcceptedEntry = tab.add(
                    displayName + " MT2 Accepted",
                    false)
                    .withPosition(baseColumn, mt2Row)
                    .withSize(2, 1)
                    .getEntry();

            mt2ReasonEntry = tab.add(
                    displayName + " MT2 Reason",
                    "Not processed")
                    .withPosition(baseColumn + 2, mt2Row)
                    .withSize(4, 1)
                    .getEntry();

            mt2StdDevEntry = tab.add(
                    displayName + " XY Std Dev m",
                    MAX_XY_STD_DEV_METERS)
                    .withPosition(baseColumn, mt2Row + 1)
                    .withSize(2, 1)
                    .getEntry();

            mt2TagCountEntry = tab.add(
                    displayName + " MT2 Tags",
                    0.0)
                    .withPosition(baseColumn + 2, mt2Row + 1)
                    .withSize(1, 1)
                    .getEntry();

            mt2DistanceEntry = tab.add(
                    displayName + " MT2 Distance m",
                    -1.0)
                    .withPosition(baseColumn + 3, mt2Row + 1)
                    .withSize(2, 1)
                    .getEntry();

            mt2AgeEntry = tab.add(
                    displayName + " MT2 Age s",
                    -1.0)
                    .withPosition(baseColumn + 5, mt2Row + 1)
                    .withSize(1, 1)
                    .getEntry();

            mt2OmegaEntry = tab.add(
                    displayName + " Omega rad/s",
                    0.0)
                    .withPosition(baseColumn, mt2Row + 2)
                    .withSize(2, 1)
                    .getEntry();

            mt2TranslationErrorEntry = tab.add(
                    displayName + " Pose Error m",
                    -1.0)
                    .withPosition(baseColumn + 2, mt2Row + 2)
                    .withSize(2, 1)
                    .getEntry();

            mt2XEntry = tab.add(
                    displayName + " MT2 X",
                    0.0)
                    .withPosition(baseColumn + 4, mt2Row + 2)
                    .withSize(1, 1)
                    .getEntry();

            mt2YEntry = tab.add(
                    displayName + " MT2 Y",
                    0.0)
                    .withPosition(baseColumn + 5, mt2Row + 2)
                    .withSize(1, 1)
                    .getEntry();

            mt2LastAcceptedTimeEntry = tab.add(
                    displayName + " Last Accepted",
                    -1.0)
                    .withPosition(baseColumn, mt2Row + 3)
                    .withSize(2, 1)
                    .getEntry();
        }
    }

    private static final class SeedCandidate {
        private final VisionCamera camera;
        private final LimelightHelpers.PoseEstimate estimate;

        private SeedCandidate(
                VisionCamera camera,
                LimelightHelpers.PoseEstimate estimate) {
            this.camera = camera;
            this.estimate = estimate;
        }
    }
}
