package frc.robot.CatzSubsystems.CatzShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.Utilities.Setpoint;

public class AimCalculations {
    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public static Setpoint calculateHubTrackingSetpoint() {
        return calculateTurretTrackingSetpoint(FieldConstants.getHubLocation());
    }

    public static Setpoint calculateTurretTrackingSetpoint(Translation2d target){
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();

        Translation2d hubDirection = target.minus(CatzTurret.Instance.getFieldToTurret());

        double targetRads = hubDirection.getAngle().minus(fieldToRobot.getRotation()).minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    public static double getFutureDistance() {
        Pose2d robotPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds robotVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds();
        double robotAngle = robotPose.getRotation().getRadians();

        double cosRobotAngle = Math.cos(robotAngle);
        double sinRobotAngle = Math.sin(robotAngle);

        double turretVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (TurretConstants.TURRET_OFFSET.getY() * cosRobotAngle
                                - TurretConstants.TURRET_OFFSET.getX() * sinRobotAngle);
        double turretVelocityY = robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (TurretConstants.TURRET_OFFSET.getX() * cosRobotAngle
                                - TurretConstants.TURRET_OFFSET.getY() * sinRobotAngle);

        Translation2d hubVelocity = new Translation2d(-turretVelocityX, -turretVelocityY); // imagine the hub moving
                                                                                           // instead of the robot
        Translation2d robotToHub = FieldConstants.getHubLocation().minus(robotPose.getTranslation());

        return 0.0;
    }

    public static boolean readyToShoot(){
        return CatzTurret.Instance.nearPositionSetpoint() && CatzFlywheels.Instance.spunUp();
    }
}
