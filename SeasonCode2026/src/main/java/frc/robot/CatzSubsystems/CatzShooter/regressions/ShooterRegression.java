package frc.robot.CatzSubsystems.CatzShooter.regressions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzTurret.TurretConstants;
import frc.robot.Utilities.InterpolatingDouble;
import frc.robot.Utilities.InterpolatingTreeMap;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.PolynomialRegression;

public class ShooterRegression {
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression flywheelAutoAimPolynomial;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> airtimeAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression airtimeAutoAimPolynomial;

    public static double[][] flywheelRegression;
    public static double[][] airtimeRegression;

    static {

        flywheelRegression = EpsilonRegression.flywheelManualRPM;

        for (double[] pair : flywheelRegression) {
            flywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        flywheelAutoAimPolynomial = new PolynomialRegression(flywheelRegression, 2);
    }

    static {
        airtimeRegression = EpsilonRegression.airtime;

        for(double[] pair : airtimeRegression){
            airtimeAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        airtimeAutoAimPolynomial = new PolynomialRegression(airtimeRegression, 2);
    }

    private static final double HOOD_ANGLE_SLOPE = (EpsilonRegression.FARTHEST_HOOD_ANGLE[1]-EpsilonRegression.CLOSEST_HOOD_ANGLE[1]) /
                                                   (EpsilonRegression.FARTHEST_HOOD_ANGLE[0]-EpsilonRegression.CLOSEST_HOOD_ANGLE[0]);

    public static double getHoodAngle(Distance distance){
        return HOOD_ANGLE_SLOPE * (distance.in(Units.Meters) - EpsilonRegression.CLOSEST_HOOD_ANGLE[0]) + EpsilonRegression.CLOSEST_HOOD_ANGLE[1];
    }

    public static double getFutureDistance(){
        Pose2d robotPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds robotVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds();
        double robotAngle = robotPose.getRotation().getRadians();

        double cosRobotAngle = Math.cos(robotAngle);
        double sinRobotAngle = Math.sin(robotAngle);

        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_CENTER.getY() * cosRobotAngle
                        - TurretConstants.TURRET_CENTER.getX() * sinRobotAngle);
        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_CENTER.getX() * cosRobotAngle
                        - TurretConstants.TURRET_CENTER.getY() * sinRobotAngle);

        Translation2d hubVelocity = new Translation2d(-turretVelocityX, -turretVelocityY); //imagine the hub moving instead of the robot
        Translation2d robotToHub = FieldConstants.HUB_LOCATION.minus(robotPose.getTranslation());

        return 0.0;
    }

}
