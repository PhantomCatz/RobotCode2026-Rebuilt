package frc.robot.CatzSubsystems.CatzShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.Utilities.Setpoint;

public class AimCalculations {

    public static ShooterSetpoints calculateAllShooterSetpoint(){
        Translation2d fieldToTurret = CatzTurret.Instance.getFieldToTurret();
        Distance distToHub = Units.Meters.of(FieldConstants.HUB_LOCATION.minus(fieldToTurret).getNorm());

        Translation2d compensatedShootingVector = calculateCompensatedShootingVector(distToHub);

        Setpoint turretSetpoint = calculateCompensatedHubTrackingSetpoint(compensatedShootingVector);
        Setpoint hoodSetpoint = ShooterRegression.getHoodSetpoint(distToHub);
        Setpoint flywheelSetpoint = getCompensatedFlywheelSetpoint(compensatedShootingVector);
        return new ShooterSetpoints(turretSetpoint, hoodSetpoint, flywheelSetpoint);
    }
    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public static Setpoint calculateHubTrackingSetpoint() {
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();

        Translation2d hubDirection = FieldConstants.HUB_LOCATION.minus(CatzTurret.Instance.getFieldToTurret());

        double targetRads = hubDirection.getAngle().minus(fieldToRobot.getRotation()).minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    /**
     * Calculates the required velocity vector for the shooter flywheel/turret
     * to compensate for the indexer feed velocity.
     * * @return Translation2d where x/y represents the shooter's velocity vector
     * (in RPS) relative to the robot.
     */
    private static Translation2d calculateCompensatedShootingVector(Distance distance) {
        // 1. Get positions
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d fieldToTurret = CatzTurret.Instance.getFieldToTurret();

        // 2. Calculate the vector pointing to the Hub
        Translation2d robotToHubVec = FieldConstants.HUB_LOCATION.minus(fieldToTurret);
        double robotDistToHub = distance.in(Units.Meters);
        // 3. Determine the "Goal Magnitude" (Total Exit Velocity Needed)
        // The regression gives the Flywheel RPS needed for a straight shot.
        // The ACTUAL ball speed in that test was (Flywheel_RPS + Feeder_Speed).
        double regressionRPS = ShooterRegression.getShooterRPSFromRegression(robotDistToHub);
        double feederSpeedRPS = FlywheelConstants.VDEXER_FEED_COMPENSATION_NORM;
        double totalRequiredSpeed = regressionRPS + feederSpeedRPS;

        // 4. Create the Goal Vector (Robot Relative)
        // This represents the path the ball must take through the air.
        // We take the direction to the hub (rotated to be robot-relative) and scale it
        // to the totalRequiredSpeed.
        Translation2d goalVector = robotToHubVec
                .rotateBy(fieldToRobot.getRotation().unaryMinus()) // Transform Field -> Robot
                .div(robotDistToHub) // Normalize to unit vector
                .times(totalRequiredSpeed); // Scale to the physics-true speed

        // 5. Get the Feeder Vector (Robot Relative)
        // This is the bias we need to cancel out.
        Translation2d feedVector = FlywheelConstants.VDEXER_FEED_COMPENSATION;

        // 6. Calculate the Shoot Vector
        // The Law of Velocities: V_goal = V_shooter + V_feed
        // Therefore: V_shooter = V_goal - V_feed
        Translation2d shootVector = goalVector.minus(feedVector);

        return shootVector;
    }

    private static Setpoint getCompensatedFlywheelSetpoint(Translation2d shootVector) {
        // The length of the vector is the speed the flywheel must run
        // to achieve the resultant vector after the feeder pushes it.
        return Setpoint.withVelocitySetpoint(shootVector.getNorm());
    }

    private static Setpoint calculateCompensatedHubTrackingSetpoint(Translation2d shootVector) {
        // 2. Extract the angle from the vector
        // This angle includes the offset needed to fight the feeder's push.
        double targetRads = shootVector.getAngle().getRadians();

        double currentRads = CatzTurret.Instance.getLatencyCompensatedPosition() * 2 * Math.PI;

        double angleError = targetRads - currentRads;
        angleError = MathUtil.angleModulus(angleError);

        Logger.recordOutput("Turret/TargetRads", targetRads);
        Logger.recordOutput("Turret/CompensatedVector", shootVector);

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(currentRads + angleError));
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
        Translation2d robotToHub = FieldConstants.HUB_LOCATION.minus(robotPose.getTranslation());

        return 0.0;
    }

    public record ShooterSetpoints(Setpoint turretSetpoint, Setpoint hoodSetpoint, Setpoint flywheelSetpoint){}
}
