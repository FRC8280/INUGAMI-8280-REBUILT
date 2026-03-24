package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShootingMath {
    private ShootingMath() {}

    /**
     * Calculates the field-relative heading the robot should face to shoot on the move.
     *
     * Assumptions:
     * - targetPos is a field-relative Translation2d
     * - robotPose is field-relative
     * - fieldRelativeSpeeds is field-relative robot velocity
     * - projectileSpeedMps is the effective horizontal speed of the game piece
     *
     * This compensates for robot translation by "leading" the shot.
     */
    public static Rotation2d calculateMovingShotAngle(
            Pose2d robotPose,
            Translation2d targetPos,
            ChassisSpeeds fieldRelativeSpeeds,
            double projectileSpeedMps) {

        Translation2d robotPos = robotPose.getTranslation();
        Translation2d toTarget = targetPos.minus(robotPos);

        double distance = toTarget.getNorm();
        if (distance < 1e-6 || projectileSpeedMps <= 1e-6) {
            return toTarget.getAngle();
        }

        // First-pass time of flight
        double tof = distance / projectileSpeedMps;

        // Shift the target backward by how far the robot will move during flight.
        // Equivalent to aiming where the target appears relative to your moving frame.
        Translation2d robotMotionDuringFlight = new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond * tof,
                fieldRelativeSpeeds.vyMetersPerSecond * tof
        );

        Translation2d compensatedVector = toTarget.minus(robotMotionDuringFlight);

        // Optional refinement pass for better accuracy
        double compensatedDistance = compensatedVector.getNorm();
        if (compensatedDistance > 1e-6) {
            double refinedTof = compensatedDistance / projectileSpeedMps;

            robotMotionDuringFlight = new Translation2d(
                    fieldRelativeSpeeds.vxMetersPerSecond * refinedTof,
                    fieldRelativeSpeeds.vyMetersPerSecond * refinedTof
            );

            compensatedVector = toTarget.minus(robotMotionDuringFlight);
        }

        return compensatedVector.getAngle();
    }

    /**
     * Utility if your drivetrain velocities are robot-relative and you need field-relative.
     */
    public static ChassisSpeeds robotRelativeToFieldRelative(
            ChassisSpeeds robotRelativeSpeeds,
            Rotation2d robotHeading) {

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                robotHeading
        );
    }
}