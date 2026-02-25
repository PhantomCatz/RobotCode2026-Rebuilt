package frc.robot.CatzSubsystems.CatzShooter.regressions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.PolynomialRegression;
import frc.robot.Utilities.Setpoint;

public class ShooterRegression {

    // Enum to select which regression to use
    public enum RegressionMode {
        HUB(0.067, Units.Degrees.of(4.0), Units.Degrees.of(4.0)), //percent threshold Hdegrees Vdegrees
        OVER_TRENCH_HOARD(0.2, Units.Degrees.of(5.0), Units.Degrees.of(5.0)),
        OVER_NET_HOARD(0.3, Units.Degrees.of(5.0), Units.Degrees.of(5.0)),
        OPP_HOARD(0.4, Units.Degrees.of(5.0), Units.Degrees.of(5.0));

        private double flywheelPercentThreshold;
                Angle turretDegThreshold, hoodDegThreshold;
        public double getFlywheelPercentThreshold() {
            return flywheelPercentThreshold;
        }
        public Angle getTurretDegThreshold() {
            return turretDegThreshold;
        }
        public Angle getHoodDegThreshold() {
            return hoodDegThreshold;
        }
        RegressionMode(double flywheelPercentThreshold, Angle turretDegThreshold, Angle hoodDegThreshold) {
            this.flywheelPercentThreshold = flywheelPercentThreshold;
            this.turretDegThreshold       = turretDegThreshold;
            this.hoodDegThreshold         = hoodDegThreshold;
        }
    }

    public static boolean kUseFlywheelPolynomial = true;

    // -------------------------------------------------------------------------
    // Tunable Numbers (For debugging/tuning live)
    // -------------------------------------------------------------------------
    public static final LoggedTunableNumber TUNABLE_HOOD_ANGLE_MIN = new LoggedTunableNumber("Regression/hood angle min", EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB[1]);
    public static final LoggedTunableNumber TUNABLE_HOOD_DIST_MIN = new LoggedTunableNumber("Regression/hood dist min", EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB[0]);

    public static final LoggedTunableNumber TUNABLE_HOOD_ANGLE_MAX = new LoggedTunableNumber("Regression/hood angle max", EpsilonRegression.FARTHEST_HOOD_ANGLE_HUB[1]);
    public static final LoggedTunableNumber TUNABLE_HOOD_DIST_MAX = new LoggedTunableNumber("Regression/hood dist max", EpsilonRegression.FARTHEST_HOOD_ANGLE_HUB[0]);

    public static final LoggedTunableNumber TUNABLE_DIST = new LoggedTunableNumber("Regression/Dist", 0.0);

    // -------------------------------------------------------------------------
    // Maps & Polynomials
    // -------------------------------------------------------------------------

    // --- Hub ---
    public static InterpolatingDoubleTreeMap hubFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression hubFlywheelPolynomial;

    public static InterpolatingDoubleTreeMap closeHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression overTrenchHoardPolynomial;

    public static InterpolatingDoubleTreeMap farHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression overNetHoardPolynomial;

    public static InterpolatingDoubleTreeMap oppHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression oppHoardPolynomial;

    public static PolynomialRegression airtimeHubInversePoly;
    public static PolynomialRegression airtimeOverTrenchHoardInversePoly;
    public static PolynomialRegression airtimeOverNetHoardInversePoly;
    public static PolynomialRegression airtimeOppHoardInversePoly;

    public static final double[] airtimeHubCoeffs = new double[3];
    public static final double[] airtimeOverTrenchHoardCoeffs = new double[3];
    public static final double[] airtimeOverNetHoardCoeffs = new double[3];
    public static final double[] airtimeOppHoardCoeffs = new double[3];

    private static final double HUB_HOOD_SLOPE;
    private static final double OVER_TRENCH_HOARD_HOOD_SLOPE;
    private static final double OVER_NET_HOARD_HOOD_SLOPE;
    private static final double OPP_HOARD_HOOD_SLOPE;

    static {
        hubFlywheelPolynomial  = loadRegression(EpsilonRegression.flywheelHubRPS, hubFlywheelMap);
        overTrenchHoardPolynomial   = loadRegression(EpsilonRegression.flywheelOverTrenchHoardRPS, closeHoardFlywheelMap);
        overNetHoardPolynomial     = loadRegression(EpsilonRegression.flywheelOverNetHoardRPS, farHoardFlywheelMap);
        oppHoardPolynomial     = loadRegression(EpsilonRegression.flywheelOppHoardRPS, oppHoardFlywheelMap);

        airtimeHubInversePoly = createInverseAirtimePoly(EpsilonRegression.airtimeHub);
        airtimeOverTrenchHoardInversePoly = createInverseAirtimePoly(EpsilonRegression.airtimeOverTrenchHoard);
        airtimeOverNetHoardInversePoly = createInverseAirtimePoly(EpsilonRegression.airtimeOverNetHoard);
        airtimeOppHoardInversePoly = createInverseAirtimePoly(EpsilonRegression.airtimeOppHoard);

        populateCoeffs(airtimeHubInversePoly, airtimeHubCoeffs);
        populateCoeffs(airtimeOverTrenchHoardInversePoly, airtimeOverTrenchHoardCoeffs);
        populateCoeffs(airtimeOverNetHoardInversePoly, airtimeOverNetHoardCoeffs);
        populateCoeffs(airtimeOppHoardInversePoly, airtimeOppHoardCoeffs);

        HUB_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB, EpsilonRegression.FARTHEST_HOOD_ANGLE_HUB);
        OVER_TRENCH_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_TRENCH_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_OVER_TRENCH_HOARD);
        OVER_NET_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_NET_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_OVER_NET_HOARD);
        OPP_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_OPP_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_OPP_HOARD);
    }

    private static PolynomialRegression loadRegression(double[][] data, InterpolatingDoubleTreeMap map) {
        for (double[] pair : data) {
            map.put(pair[0], pair[1]);
        }
        return new PolynomialRegression(data, 2);
    }

    private static PolynomialRegression createInverseAirtimePoly(double[][] originalData) {
        double[][] invArr = new double[originalData.length][2];
        int i = 0;
        for (double[] row : originalData) {
            invArr[i][0] = row[1];
            invArr[i][1] = row[0];
            i++;
        }
        return new PolynomialRegression(invArr, 1);
    }

    private static void populateCoeffs(PolynomialRegression poly, double[] targetArray) {
        double a = poly.beta(1);
        double b = poly.beta(0);

        targetArray[0] = a * a;
        targetArray[1] = 2 * a * b;
        targetArray[2] = b * b;
    }

    private static double calculateSlope(double[] min, double[] max) {
        return (max[1] - min[1]) / (max[0] - min[0]);
    }

    public static double[] getAirtimeCoeffs(RegressionMode mode) {
        switch (mode) {
            case OVER_TRENCH_HOARD: return airtimeOverTrenchHoardCoeffs;
            case OVER_NET_HOARD:   return airtimeOverNetHoardCoeffs;
            case OPP_HOARD:   return airtimeOppHoardCoeffs;
            case HUB:
            default:          return airtimeHubCoeffs;
        }
    }

    public static Setpoint getShooterSetpoint(Distance range, RegressionMode mode) {
        double rps = 0.0;
        double distMeters = range.in(Units.Meters);

        if (kUseFlywheelPolynomial) {
            switch (mode) {
                case HUB:         rps = hubFlywheelPolynomial.predict(distMeters); break;
                case OVER_TRENCH_HOARD: rps = overTrenchHoardPolynomial.predict(distMeters);  break;
                case OVER_NET_HOARD:   rps = overNetHoardPolynomial.predict(distMeters);    break;
                case OPP_HOARD:   rps = oppHoardPolynomial.predict(distMeters);    break;
            }
        } else {
            switch (mode) {
                case HUB:         rps = hubFlywheelMap.get(distMeters); break;
                case OVER_TRENCH_HOARD: rps = closeHoardFlywheelMap.get(distMeters); break;
                case OVER_NET_HOARD:   rps = farHoardFlywheelMap.get(distMeters); break;
                case OPP_HOARD:   rps = oppHoardFlywheelMap.get(distMeters); break;
            }
        }

        return Setpoint.withVelocitySetpointVoltage(rps);
    }

    public static Setpoint getHoodSetpoint(Distance range, RegressionMode mode) {
        double distMeters = range.in(Units.Meters);
        double angle = 0.0;

        double slope = 0.0;
        double yInterceptAngle = 0.0;
        double xInterceptDist = 0.0;

        switch (mode) {
            case HUB:
                slope = HUB_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB[0];
                break;
            case OVER_TRENCH_HOARD:
                slope = OVER_TRENCH_HOARD_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_TRENCH_HOARD[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_TRENCH_HOARD[0];
                break;
            case OVER_NET_HOARD:
                slope = OVER_NET_HOARD_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_NET_HOARD[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_OVER_NET_HOARD[0];
                break;
            case OPP_HOARD:
                slope = OPP_HOARD_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_OPP_HOARD[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_OPP_HOARD[0];
                break;
        }

        // point slope form: y = m(x - x1) + y1
        angle = slope * (distMeters - xInterceptDist) + yInterceptAngle;

        angle = MathUtil.clamp(angle,
            HoodConstants.HOOD_ZERO_POS.in(Units.Degrees),
            HoodConstants.HOOD_MAX_POS.in(Units.Degrees)
        );

        return Setpoint.withMotionMagicSetpoint(Units.Degrees.of(angle));
    }

    // -------------------------------------------------------------------------
    // Tunable / Debug
    // -------------------------------------------------------------------------
    public static double getHoodAngleTunable(Distance distance) {
        double minAngle = TUNABLE_HOOD_ANGLE_MIN.get();
        double minDist = TUNABLE_HOOD_DIST_MIN.get();
        double maxAngle = TUNABLE_HOOD_ANGLE_MAX.get();
        double maxDist = TUNABLE_HOOD_DIST_MAX.get();

        double slope = (maxAngle - minAngle) / (maxDist - minDist);
        return slope * (distance.in(Units.Meters) - minDist) + minAngle;
    }

    public static double getHoodAngleTunable() {
        return getHoodAngleTunable(Units.Meters.of(TUNABLE_DIST.get()));
    }
}
