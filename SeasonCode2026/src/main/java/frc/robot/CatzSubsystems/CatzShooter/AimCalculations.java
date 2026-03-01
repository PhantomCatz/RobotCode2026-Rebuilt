package frc.robot.CatzSubsystems.CatzShooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.Setpoint;


import org.littletonrobotics.junction.Logger;

public class AimCalculations {
    private static final double phaseDelay = 0.0;
    private static final LoggedTunableNumber delayy = new LoggedTunableNumber("phase delay", phaseDelay);

    public enum HoardTargetType {
        RELATIVE_CLOSE,
        RELATIVE_FAR,
        ABSOLUTE_LEFT,
        ABSOLUTE_RIGHT
    }

    /**
     *
     * @param distToCenter
     * @return Angle in radians
     */
    public static double calculateHoodBisectorAngle(double distToCenter) {
        double distToRim = distToCenter - FieldConstants.HUB_RIM_RADIUS.in(Units.Meter);

        double slopeAngle = Math.atan2(FieldConstants.HEIGHT_DIFF, distToRim);
        return Math.PI/2.0 - (slopeAngle + (Math.PI / 2.0)) / 2.0;
    }

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public static Setpoint calculateHubTrackingSetpoint() {
        return calculateTurretTrackingSetpoint(FieldConstants.getHubLocation());
    }

    public static Setpoint calculateTurretTrackingSetpoint(Translation2d target) {
        Translation2d hubDirection = target.minus(CatzTurret.Instance.getFieldToTurret());
        double targetRads = hubDirection.getAngle().minus(CatzRobotTracker.Instance.getEstimatedPose().getRotation())
                .minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();
        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    public static Setpoint calculateTurretTrackingSetpoint(Translation2d target, Translation2d predictedTurretPose) {
        Translation2d hubDirection = target.minus(predictedTurretPose);
        double targetRads = hubDirection.getAngle().minus(CatzRobotTracker.Instance.getEstimatedPose().getRotation())
                .minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();
        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    public static Translation2d getCornerHoardingTarget(HoardTargetType targetType) {
        Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
        Translation2d targetPos = FieldConstants.getRightCornerHoardLocation();
        boolean shouldMirror = false;

        boolean isLeftHalf = turretPos.getY() >= FieldConstants.fieldYHalf;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            isLeftHalf = turretPos.getY() <= FieldConstants.fieldYHalf;
        }

        switch (targetType) {
            case RELATIVE_CLOSE:
                shouldMirror = isLeftHalf;
                break;
            case RELATIVE_FAR:
                shouldMirror = !isLeftHalf;
                break;
            case ABSOLUTE_LEFT:
                shouldMirror = true;
                break;
            case ABSOLUTE_RIGHT:
                shouldMirror = false;
                break;
        }

        if (shouldMirror) {
            targetPos = new Translation2d(targetPos.getX(), FieldConstants.fieldWidth - targetPos.getY());
        }
        return targetPos;
    }

    public static boolean doesTrajectoryCrossNet(Translation2d turretPos, Translation2d targetPos) {
        Translation2d netBase = FieldConstants.getNetLocation();
        double netX = netBase.getX();
        double netYMin = FieldConstants.fieldYHalf - (FieldConstants.NET_LENGTH_HALF);
        double netYMax = FieldConstants.fieldYHalf + (FieldConstants.NET_LENGTH_HALF);

        if ((turretPos.getX() <= netX && targetPos.getX() >= netX) ||
                (turretPos.getX() >= netX && targetPos.getX() <= netX)) {

            double slope = (targetPos.getY() - turretPos.getY()) / (targetPos.getX() - turretPos.getX());
            double yInt = turretPos.getY() + slope * (netX - turretPos.getX());

            return yInt >= netYMin && yInt <= netYMax;
        }
        return false;
    }

    public static RegressionMode getHoardRegressionMode(Translation2d targetPos) {
        if (doesTrajectoryCrossNet(CatzTurret.Instance.getFieldToTurret(), targetPos)) {
            return RegressionMode.OVER_NET_HOARD;
        }
        return RegressionMode.OVER_TRENCH_HOARD;
    }

    public static Translation2d calculateAndGetPredictedTargetLocation(Translation2d baseTarget, RegressionMode mode,
            Pose2d predictedRobotPose, Translation2d predictedTurretPose) {
        Translation2d targetVelocity = getTargetVelocityRelativeToRobot(predictedRobotPose);
        double futureAirtime = getFutureShootAirtime(predictedTurretPose, targetVelocity, baseTarget, mode);
        return baseTarget.plus(targetVelocity.times(futureAirtime));
    }

    private static Translation2d getTargetVelocityRelativeToRobot(Pose2d predictedRobotPose) {
        ChassisSpeeds currentVelocity = CatzRobotTracker.Instance.getFieldRelativeChassisSpeeds();

        double turretRadialAngle = (predictedRobotPose.getRotation().plus(TurretConstants.TURRET_RADIAL_ANGLE))
                .getRadians();

        double turretXVelocity = -Math.sin(turretRadialAngle) * TurretConstants.TURRET_DIST_TO_CENTER
                * currentVelocity.omegaRadiansPerSecond + currentVelocity.vxMetersPerSecond;
        double turretYVelocity = Math.cos(turretRadialAngle) * TurretConstants.TURRET_DIST_TO_CENTER
                * currentVelocity.omegaRadiansPerSecond + currentVelocity.vyMetersPerSecond;

        return new Translation2d(-turretXVelocity, -turretYVelocity);
    }

    private static double getFutureShootAirtime(Translation2d fieldToTurret, Translation2d targetVelocity,
            Translation2d targetPos,
            RegressionMode mode) {
        Translation2d targetToTurret = fieldToTurret.minus(targetPos);
        double distToTarget = targetToTurret.getNorm();

        double turretTargetRadians = Math.abs(
                MathUtil.angleModulus(targetToTurret.getAngle().getRadians() - targetVelocity.getAngle().getRadians()));
        double[] regCoeffs = ShooterRegression.getAirtimeCoeffs(mode);

        double targetSpeed = targetVelocity.getNorm();

        double a = targetSpeed * targetSpeed - regCoeffs[0];
        double b = -2 * targetSpeed * distToTarget * Math.cos(turretTargetRadians) - regCoeffs[1];
        double c = distToTarget * distToTarget - regCoeffs[2];

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return 0.0;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double biggerRoot = (-b + sqrtDiscriminant) / (2 * a);
        double smallerRoot = (-b - sqrtDiscriminant) / (2 * a);

        if (smallerRoot > 0) {
            return smallerRoot;
        }

        if (biggerRoot > 0) {
            return biggerRoot;
        }

        return 0.0;
    }

    public static Pose2d getPredictedRobotPose() {
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds robotVelocity = CatzRobotTracker.Instance.getRobotRelativeChassisSpeeds();

        Twist2d twist = new Twist2d(
                robotVelocity.vxMetersPerSecond * delayy.get(),
                robotVelocity.vyMetersPerSecond * delayy.get(),
                robotVelocity.omegaRadiansPerSecond * delayy.get());

        return currentPose.exp(twist);
    }

    public static boolean readyToShoot() {
        Logger.recordOutput("Turret Ready", CatzTurret.Instance.nearPositionSetpoint());
        Logger.recordOutput("Hood Ready", CatzHood.Instance.nearPositionSetpoint());
        Logger.recordOutput("Flywheel Ready", CatzFlywheels.Instance.spunUp());
        return CatzTurret.Instance.nearPositionSetpoint() && CatzFlywheels.Instance.spunUp();
    }
}
