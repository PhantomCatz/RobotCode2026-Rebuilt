package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;

public class ApriltagScanningIOMovable implements ApriltagScanningIO {

    private Pose2d latestEstimate = new Pose2d();
    private Time latestEstimateTime = Units.Seconds.of(0.0);
    private int latestEstimateNumTags = 0;
    private LimelightConfig config = new LimelightConfig();

    public ApriltagScanningIOMovable(LimelightConfig config) {
        this.config = config;

    }

    @Override
    public void update() {
        updateGyroWithTurret();

        double robotOmegaDegPerSec = Math.toDegrees(CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond);

        double turretOmegaDegPerSec = CatzTurret.Instance.getVelocity().in(Units.DegreesPerSecond);

        double cameraFieldOmega = robotOmegaDegPerSec + turretOmegaDegPerSec;

        if (Math.abs(cameraFieldOmega) > 300.0) {
            System.out.println("Limelight turning too fast!!!!!");
            return;
        }
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        setLatestEstimate(estimate, 1);
    }

    private void updateGyroWithTurret() {
        Rotation2d robotYaw = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
        Rotation2d turretYaw = Rotation2d.fromRotations(CatzTurret.Instance.getLatencyCompensatedPosition())
                .plus(TurretConstants.TURRET_ROTATION_OFFSET);

        double totalCameraHeading = (robotYaw.plus(turretYaw)).getDegrees();

        // TODO add yaw rate for better accuracy?
        LimelightHelpers.SetRobotOrientation(config.name, totalCameraHeading, 0, 0, 0, 0, 0);
    }

    @Override
    public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
        if (poseEstimate == null)
            return;

        latestEstimateNumTags = poseEstimate.tagCount;

        if (poseEstimate.tagCount >= minTagNum) {
            /*
             * The previous method of just updating turret's position relative to the robot every loop doesnt work because
             * there is latency between the limelight capturing the image data and actually processing it. If it captures
             * an image when turret's at 10 degrees and the turret rotates between the time of capture and time of process
             * and we send a new turret position during that time, it will calculate the robot position with an old image but
             * with a new turret position.
             *
             * We make the turret a "mini robot" on top of the real robot and reverse calculate the robot's position based off of the turret's position.
             * So we have a static limelight offset relative to the turret. The limelight calculates the turret's position correctly,
             * and using a time lookup table grabbing the past turret angle that matches the time of turret position calculations,
             * we can reverse calculate the robot position with correct latency correction.
             */
            double timestamp = poseEstimate.timestampSeconds;

            Pose2d turretPoseFieldSpace = poseEstimate.pose;
            Logger.recordOutput("Turret Pose Field Space", turretPoseFieldSpace);

            Rotation2d pastTurretRot = new Rotation2d(CatzTurret.Instance.getAngleAtTime(timestamp)).plus(TurretConstants.TURRET_ROTATION_OFFSET);

            Transform2d robotToTurret = new Transform2d(
                    TurretConstants.TURRET_OFFSET,
                    pastTurretRot);

            Pose2d robotPoseFieldSpace = turretPoseFieldSpace.transformBy(robotToTurret.inverse());

            latestEstimate = robotPoseFieldSpace;
            latestEstimateTime = Units.Seconds.of(timestamp);

            CatzRobotTracker.Instance.addVisionObservation(
                    new VisionObservation(
                            config.name,
                            robotPoseFieldSpace,
                            timestamp,
                            config.aprilTagVisionStdDevs.times(poseEstimate.avgTagDist)));
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTargets = latestEstimateNumTags > 0;
        inputs.calculatedPose = latestEstimate;
    }

    public Pose2d getLatestEstimate() {
        return latestEstimate;
    }

    public Time getLatestEstimateTime() {
        return latestEstimateTime;
    }

    public int getLatestEstimateNumTags() {
        return latestEstimateNumTags;
    }

    public LimelightConfig getConfig() {
        return config;
    }

    @Override
    public int getNumTags() {
        return latestEstimateNumTags;
    }
}
