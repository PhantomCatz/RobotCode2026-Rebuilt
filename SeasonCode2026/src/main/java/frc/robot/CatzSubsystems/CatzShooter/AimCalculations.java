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
import frc.robot.Utilities.Setpoint;
import io.grpc.InternalChannelz.RootChannelList;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

public class AimCalculations {
    private static final double phaseDelay = 0.03;
    private static LaguerreSolver solver = new LaguerreSolver();

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public static Setpoint calculateHubTrackingSetpoint() {
        return calculateTurretTrackingSetpoint(FieldConstants.getHubLocation());
    }

    public static Setpoint calculateCornerTrackingSetpoint(){
        return calculateTurretTrackingSetpoint(getCornerHoardingTarget(true));
    }

    public static Setpoint calculateTurretTrackingSetpoint(Translation2d target) {
        Translation2d hubDirection = target.minus(CatzTurret.Instance.getFieldToTurret());

        double targetRads = hubDirection.getAngle().minus(CatzRobotTracker.Instance.getEstimatedPose().getRotation())
                .minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    public static Translation2d getCornerHoardingTarget(boolean isCloseCornerHoarding) {
        Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
        Translation2d targetPos = FieldConstants.getRightCornerHoardLocation();

        boolean shouldMirror = false;

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (turretPos.getY() >= FieldConstants.fieldYHalf) {
                shouldMirror = true;
            }
        } else {
            if (turretPos.getY() <= FieldConstants.fieldYHalf) {
                shouldMirror = true;
            }
        }

        if (!isCloseCornerHoarding) {
            shouldMirror = !shouldMirror;
        }

        if (shouldMirror) {
            targetPos = new Translation2d(targetPos.getX(), FieldConstants.fieldWidth - targetPos.getY());
        }
        return targetPos;
    }

    public static Translation2d getPredictedHubLocation() {
        Pose2d robotPose = getPredictedRobotPose();
        Translation2d hubVelocity = getHubVelocity(robotPose);
        double futureAirtime = getFutureShootAirtime(robotPose, hubVelocity);
        return hubVelocity.times(futureAirtime);
    }

    /**
     * Calculates the hub's velocity vector relative to the turret pretending as if
     * the robot is stationary and the hub is moving.
     *
     * @return
     */
    private static Translation2d getHubVelocity(Pose2d robotPose) {
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

        return new Translation2d(-turretVelocityX, -turretVelocityY);
    }

    /**
     * Calculates the airtime that the ball will take when shot at the predicted future location of the hub
     *
     * We avoid the problem of needing to iteratively search the potential solution by approximating the inverse airtime
     * function as a second degree polynomial.
     *
     * @return The predicted future airtime of the ball. If no solution is found, return 0.
     */
    private static double getFutureShootAirtime(Pose2d robotPose, Translation2d hubVelocity) {
        Translation2d fieldToTurret = CatzTurret.Instance.getFieldToTurret(robotPose);
        Translation2d hubToTurret = fieldToTurret.minus(FieldConstants.getHubLocation());

        double distToHub = hubToTurret.getNorm();

        // calculate the angle between the hub velocity and hub displacement vectors
        double turretHubRadians = Math
                .abs(MathUtil.angleModulus(hubToTurret.getAngle().getRadians() - hubVelocity.getAngle().getRadians()));

        //get the coefficient terms of the inverse airtime polynomial
        double regressionATerm = ShooterRegression.airtimeRegA;
        double regressionBTerm = ShooterRegression.airtimeRegB;
        double regressionCTerm = ShooterRegression.airtimeRegC;
        double regressionDTerm = ShooterRegression.airtimeRegD;
        double regressionETerm = ShooterRegression.airtimeRegE;

        double hubSpeed = hubVelocity.getNorm();
        double a = regressionATerm;
        double b = regressionBTerm;
        double c = regressionCTerm - hubSpeed*hubSpeed;
        double d = 2*hubSpeed*distToHub*Math.cos(turretHubRadians) + regressionDTerm;
        double e = regressionETerm - distToHub*distToHub;

        double[] coeffs = {e, d, c, b, a};
        PolynomialFunction poly = new PolynomialFunction(coeffs);

        Complex[] roots = solver.solveAllComplex(coeffs, 0);

        double minNonnegativeRealRoot = 9999999.9;

        for (Complex r : roots) {
            if (Math.abs(r.getImaginary()) < 1e-6 && r.getReal() > 0.0) {
                minNonnegativeRealRoot = Math.min(minNonnegativeRealRoot, r.getReal());
            }
        }

        if (minNonnegativeRealRoot != 9999999.9) {
            return minNonnegativeRealRoot;
        }

        return 0.0; // both roots negative (if that's even possible)
    }

    private static Pose2d getPredictedRobotPose() {
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds robotVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        Twist2d twist = new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay);

        return currentPose.exp(twist);
    }

    public static boolean readyToShoot() {
        return CatzTurret.Instance.nearPositionSetpoint() && CatzHood.Instance.nearPositionSetpoint()
                && CatzFlywheels.Instance.spunUp();
    }
}
