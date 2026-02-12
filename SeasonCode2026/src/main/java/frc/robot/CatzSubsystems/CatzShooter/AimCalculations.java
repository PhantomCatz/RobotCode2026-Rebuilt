package frc.robot.CatzSubsystems.CatzShooter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.Utilities.Setpoint;

public class AimCalculations {
    private static final double phaseDelay = 0.03;

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public static Setpoint calculateHubTrackingSetpoint() {
        return calculateTurretTrackingSetpoint(FieldConstants.getHubLocation());
    }

    public static Setpoint calculateTurretTrackingSetpoint(Translation2d target){
        Translation2d hubDirection = target.minus(CatzTurret.Instance.getFieldToTurret());

        double targetRads = hubDirection.getAngle().minus(CatzRobotTracker.Instance.getEstimatedPose().getRotation()).minus(TurretConstants.TURRET_ROTATION_OFFSET).getRadians();

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    // public static Setpoint calculateCornerHoardingSetpoint(){
    //     Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
    //     Translation2d targetPos = FieldConstants.getRightCornerHoardLocation(); //first get the right corner

    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         if(turretPos.getY() >= FieldConstants.fieldYHalf){
    //             targetPos = new Translation2d(targetPos.getX(), FieldConstants.fieldWidth - targetPos.getY());
    //         }
    //     }else{
    //         if(turretPos.getY() <= FieldConstants.fieldYHalf){
    //             targetPos = new Translation2d(targetPos.getX(), FieldConstants.fieldWidth - targetPos.getY());
    //         }
    //     }
    // }

// public static AimingParameters getAimingParameters(Rotation2d currentTurretAngle) {
//         // 1. Predict & Calculate Target (Same as before)
//         Pose2d futureRobotPose = getPredictedRobotPose(kLatencySeconds);
//         Translation2d relativeHubVelocity = getHubVelocity(futureRobotPose);
//         double airtime = getFutureShootAirtime(relativeHubVelocity);

//         Translation2d futureTurretPos = futureRobotPose.getTranslation().plus(
//             TurretConstants.TURRET_OFFSET.rotateBy(futureRobotPose.getRotation())
//         );
//         Translation2d targetVector = FieldConstants.getHubLocation()
//                 .minus(futureTurretPos)
//                 .plus(relativeHubVelocity.times(airtime));

//         // 2. Calculate ideal angle in range [-PI, PI]
//         Rotation2d targetAngle = targetVector.getAngle();
//         double targetRadians = targetAngle.getRadians();

//         // 3. LOGIC: Handle Hard Stops & Robot Turn Assist
//         double clampedTurretGoal = targetRadians;
//         Double robotTurnCmd = null;

//         // Check if we are approaching the positive or negative limit (180 - 5 = 175 degrees)
//         boolean inPositiveDangerZone = targetRadians > (kMaxTurretAngle - kTurretHardStopBuffer);
//         boolean inNegativeDangerZone = targetRadians < (-kMaxTurretAngle + kTurretHardStopBuffer);

//         if (inPositiveDangerZone || inNegativeDangerZone) {
//              // A. CLAMP THE TURRET
//              // Don't let the turret hit the physical hard stop. Keep it at the edge.
//              // This keeps the camera looking at the target as long as possible.
//              clampedTurretGoal = MathUtil.clamp(targetRadians, -kMaxTurretAngle, kMaxTurretAngle);

//              // B. TURN THE ROBOT
//              // We need to spin the robot to bring the target back to center (0 degrees).
//              // Determine direction:
//              // If target is +179, we want to spin robot LEFT (positive omega) to make target relative angle smaller.
//              // If target is -179, we want to spin robot RIGHT (negative omega).

//              // Simple P-Controller for the Drivetrain rotation
//              // kP should be tuned (start low, e.g., 2.0)
//              double kP_Chassis = 4.0;
//              robotTurnCmd = targetRadians * kP_Chassis;
//         }

//         // 4. Calculate Turret Feedforward (Standard)
//         double r2 = targetVector.getNorm() * targetVector.getNorm();
//         double crossProduct = targetVector.getX() * relativeHubVelocity.getY()
//                             - targetVector.getY() * relativeHubVelocity.getX();
//         double turretFF = (r2 > 1e-6) ? (crossProduct / r2) : 0.0;

//         // If we are commanding the robot to turn, we must SUBTRACT that velocity from the turret
//         // so the turret stays "world stabilized" while the chassis spins beneath it.
//         if (robotTurnCmd != null) {
//             turretFF -= robotTurnCmd;
//         }

//         return new AimingParameters(
//             new Rotation2d(clampedTurretGoal),
//             turretFF,
//             robotTurnCmd
//         );
//     }

    public static Translation2d getPredictedHubLocation() {
        Translation2d hubVelocity = getHubVelocity();
        double futureAirtime = getFutureShootAirtime(hubVelocity);
        return hubVelocity.times(futureAirtime);
    }

    /**
     * Calculates the hub's velocity vector relative to the turret pretending as if the robot is stationary and the hub is moving.
     * @return
     */
    private static Translation2d getHubVelocity() {
        Pose2d robotPose = getPredictedRobotPose();
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
    private static double getFutureShootAirtime(Translation2d hubVelocity) {
        Translation2d fieldToTurret = CatzTurret.Instance.getFieldToTurret();
        Translation2d hubToTurret = fieldToTurret.minus(FieldConstants.getHubLocation());

        double distToHub = hubToTurret.getNorm();

        //calculate the angle between the hub velocity and hub displacement vectors
        double turretHubRadians = Math.abs(MathUtil.angleModulus(hubToTurret.getAngle().getRadians() - hubVelocity.getAngle().getRadians()));

        //get the coefficient terms of the inverse airtime polynomial
        double regressionATerm = ShooterRegression.airtimeRegA;
        double regressionBTerm = ShooterRegression.airtimeRegB;
        double regressionCTerm = ShooterRegression.airtimeRegC;

        double hubSpeed = hubVelocity.getNorm();
        double a = hubSpeed*hubSpeed - regressionATerm;
        double b = -2*hubSpeed*distToHub*Math.cos(turretHubRadians) - regressionBTerm;
        double c = distToHub*distToHub - regressionCTerm;
        double discriminant = b*b - 4*a*c;
        double sqrtDiscriminant = Math.sqrt(discriminant);
        if (discriminant < 0) {
            return 0.0; // no solution
        }
        double smallRoot = (-b-sqrtDiscriminant)/(2*a);
        double bigRoot = (-b+sqrtDiscriminant)/(2*a);
        if (smallRoot > 0) {
            return smallRoot;
        }
        if (bigRoot > 0) {
            return bigRoot;
        }
        return 0.0; // both roots negative (if that's even possible)
    }

    private static Pose2d getPredictedRobotPose() {
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds robotVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        Twist2d twist = new Twist2d(
            robotVelocity.vxMetersPerSecond * phaseDelay,
            robotVelocity.vyMetersPerSecond * phaseDelay,
            robotVelocity.omegaRadiansPerSecond * phaseDelay
        );

        return currentPose.exp(twist);
    }

    public static boolean readyToShoot(){
        return CatzTurret.Instance.nearPositionSetpoint() && CatzHood.Instance.nearPositionSetpoint() && CatzFlywheels.Instance.spunUp();
    }
}
