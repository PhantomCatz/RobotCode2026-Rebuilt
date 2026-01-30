package frc.robot.CatzSubsystems.CatzShooter.regressions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.PolynomialRegression;

public class ShooterRegression {
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;
    public static final LoggedTunableNumber TUNABLE_HOOD_ANGLE_MIN = new LoggedTunableNumber("Regression/hood angle min", EpsilonRegression.CLOSEST_HOOD_ANGLE[1]);
    public static final LoggedTunableNumber TUNABLE_HOOD_DIST_MIN = new LoggedTunableNumber("Regression/hood dist min", EpsilonRegression.CLOSEST_HOOD_ANGLE[0]);

    public static final LoggedTunableNumber TUNABLE_HOOD_ANGLE_MAX = new LoggedTunableNumber("Regression/hood angle max", EpsilonRegression.FARTHEST_HOOD_ANGLE[1]);
    public static final LoggedTunableNumber TUNABLE_HOOD_DIST_MAX = new LoggedTunableNumber("Regression/hood dist max", EpsilonRegression.FARTHEST_HOOD_ANGLE[0]);

    public static final LoggedTunableNumber TUNABLE_DIST = new LoggedTunableNumber("Regression/Dist", 0.0);

    public static InterpolatingDoubleTreeMap flywheelAutoAimMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression flywheelAutoAimPolynomial;

    public static InterpolatingDoubleTreeMap airtimeAutoAimMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression airtimeAutoAimPolynomial;

    // New variables for Inverse Airtime
    public static InterpolatingDoubleTreeMap airtimeInverseAutoAimMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression airtimeInverseAutoAimPolynomial;

    public static double[][] flywheelRegression;
    public static double[][] airtimeRegression;
    public static double[][] airtimeInverseRegression; // Needed to feed the polynomial constructor

    static {
        flywheelRegression = EpsilonRegression.flywheelManualRPM;

        for (double[] pair : flywheelRegression) {
            flywheelAutoAimMap.put(pair[0], pair[1]);
        }

        flywheelAutoAimPolynomial = new PolynomialRegression(flywheelRegression, 2);
    }

    static {
        airtimeRegression = EpsilonRegression.airtime;

        // Initialize the inverse array with the same size
        airtimeInverseRegression = new double[airtimeRegression.length][2];

        for (int i = 0; i < airtimeRegression.length; i++) {
            double x = airtimeRegression[i][0];
            double y = airtimeRegression[i][1];

            // Standard: Distance -> Time
            airtimeAutoAimMap.put(x, y);

            // Inverse: Time -> Distance
            // We swap the Key (x) and Value (y)
            airtimeInverseAutoAimMap.put(y, x);

            // Fill the inverse array for the Polynomial class
            airtimeInverseRegression[i][0] = y; // New X is old Y
            airtimeInverseRegression[i][1] = x; // New Y is old X
        }

        airtimeAutoAimPolynomial = new PolynomialRegression(airtimeRegression, 2);
        airtimeInverseAutoAimPolynomial = new PolynomialRegression(airtimeInverseRegression, 2);
    }

    private static final double HOOD_ANGLE_SLOPE = (EpsilonRegression.FARTHEST_HOOD_ANGLE[1]-EpsilonRegression.CLOSEST_HOOD_ANGLE[1]) /
                                                   (EpsilonRegression.FARTHEST_HOOD_ANGLE[0]-EpsilonRegression.CLOSEST_HOOD_ANGLE[0]);

    public static double getHoodAngle(Distance distance){
        double angle = HOOD_ANGLE_SLOPE * (distance.in(Units.Meters) - EpsilonRegression.CLOSEST_HOOD_ANGLE[0]) + EpsilonRegression.CLOSEST_HOOD_ANGLE[1];
        return MathUtil.clamp(angle, HoodConstants.HOOD_ZERO_POS.in(Units.Degrees), HoodConstants.HOOD_MAX_POS.in(Units.Degrees));
    }

    /**
     * Calculates the hood angle using the live TunableNumbers from the dashboard.
     * Use this during the match with the real camera distance.
     */
    public static double getHoodAngleTunable(Distance distance) {
        // 1. Get the live values from the dashboard
        double minAngle = TUNABLE_HOOD_ANGLE_MIN.get();
        double minDist = TUNABLE_HOOD_DIST_MIN.get();

        double maxAngle = TUNABLE_HOOD_ANGLE_MAX.get();
        double maxDist = TUNABLE_HOOD_DIST_MAX.get();

        // 2. Calculate Slope dynamically (Rise / Run)
        double slope = (maxAngle - minAngle) / (maxDist - minDist);

        // 3. Point-Slope Form: y = m(x - x1) + y1
        return slope * (distance.in(Units.Meters) - minDist) + minAngle;
    }

    /**
     * Debug wrapper: Calculates the angle based on the "Regression/Dist"
     * slider on the dashboard. Useful for checking the curve without moving the robot.
     */
    public static double getHoodAngleTunable() {
        return getHoodAngleTunable(Units.Meters.of(TUNABLE_DIST.get()));
    }
}
