package frc.robot.CatzSubsystems.CatzShooter.regressions;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Utilities.InterpolatingDouble;
import frc.robot.Utilities.InterpolatingTreeMap;
import frc.robot.Utilities.PolynomialRegression;

public class ShooterRegression {
    public static final double kHoodPaddingDegrees = 2;
    public static final double kShooterPaddingVelocity = 100;

    public static final double[] kPadding = {
            kShooterPaddingVelocity, kHoodPaddingDegrees};


    public static boolean kUseSmartdashboard = false;

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

    public static double getFutureDistance(Translation2d robotSpeed){
        Translation2d robotPose = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
        
        Translation2d displacementVector = 
    }

}
