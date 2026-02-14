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
        HUB(0.02, Units.Degrees.of(2.0), Units.Degrees.of(2.0)), //percent threshold Hdegrees Vdegrees
        CLOSE_HOARD(0.2, Units.Degrees.of(3.0), Units.Degrees.of(3.0)),
        FAR_HOARD(0.3, Units.Degrees.of(4.0), Units.Degrees.of(4.0)),
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

    // --- Close Corner Hoard ---
    public static InterpolatingDoubleTreeMap closeHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression closeHoardPolynomial;

    // --- Far Corner Hoard ---
    public static InterpolatingDoubleTreeMap farHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression farHoardPolynomial;

    // --- Opposite Alliance Hoard ---
    public static InterpolatingDoubleTreeMap oppHoardFlywheelMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression oppHoardPolynomial;

    // --- Airtime (Used for lead calculation) ---
    public static InterpolatingDoubleTreeMap airtimeMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression airtimePolynomial;

    public static InterpolatingDoubleTreeMap airtimeInverseMap = new InterpolatingDoubleTreeMap();
    public static PolynomialRegression airtimeInversePolynomial;


    // -------------------------------------------------------------------------
    // Pre-calculated Slopes for Linear Hood Interpolation
    // -------------------------------------------------------------------------
    private static final double HUB_HOOD_SLOPE;
    private static final double CLOSE_HOARD_HOOD_SLOPE;
    private static final double FAR_HOARD_HOOD_SLOPE;
    private static final double OPP_HOARD_HOOD_SLOPE;


    // -------------------------------------------------------------------------
    // Static Initialization
    // -------------------------------------------------------------------------
    static {
        // 1. Initialize Flywheel Regressions
        hubFlywheelPolynomial  = loadRegression(EpsilonRegression.flywheelHubRPS, hubFlywheelMap);
        closeHoardPolynomial   = loadRegression(EpsilonRegression.flywheelCloseHoardRPS, closeHoardFlywheelMap);
        farHoardPolynomial     = loadRegression(EpsilonRegression.flywheelFarHoardRPS, farHoardFlywheelMap);
        oppHoardPolynomial     = loadRegression(EpsilonRegression.flywheelOppHoardRPS, oppHoardFlywheelMap);

        // 2. Initialize Airtime Regressions (Standard & Inverse)
        double[][] airtimeArr = EpsilonRegression.airtimeHub;
        double[][] airtimeInvArr = new double[airtimeArr.length][2];

        for (int i = 0; i < airtimeArr.length; i++) {
            double dist = airtimeArr[i][0];
            double time = airtimeArr[i][1];

            airtimeMap.put(dist, time);

            // Inverse: Time -> Distance
            airtimeInverseMap.put(time, dist);
            airtimeInvArr[i][0] = time;
            airtimeInvArr[i][1] = dist;
        }

        airtimePolynomial = new PolynomialRegression(airtimeArr, 2);
        airtimeInversePolynomial = new PolynomialRegression(airtimeInvArr, 2);

        // 3. Pre-calculate Linear Hood Slopes (Rise / Run)
        HUB_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_HUB, EpsilonRegression.FARTHEST_HOOD_ANGLE_HUB);
        CLOSE_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_CLOSE_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_CLOSE_HOARD);
        FAR_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_FAR_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_FAR_HOARD);
        OPP_HOARD_HOOD_SLOPE = calculateSlope(EpsilonRegression.CLOSEST_HOOD_ANGLE_OPP_HOARD, EpsilonRegression.FARTHEST_HOOD_ANGLE_OPP_HOARD);
    }

    private static PolynomialRegression loadRegression(double[][] data, InterpolatingDoubleTreeMap map) {
        for (double[] pair : data) {
            map.put(pair[0], pair[1]);
        }
        return new PolynomialRegression(data, 2);
    }

    private static double calculateSlope(double[] min, double[] max) {
        return (max[1] - min[1]) / (max[0] - min[0]);
    }

    // -------------------------------------------------------------------------
    // Public Accessors
    // -------------------------------------------------------------------------

    public static Setpoint getShooterSetpoint(Distance range, RegressionMode mode) {
        double rps = 0.0;
        double distMeters = range.in(Units.Meters);

        // Select the correct regression source
        if (kUseFlywheelPolynomial) {
            switch (mode) {
                case HUB:         rps = hubFlywheelPolynomial.predict(distMeters); break;
                case CLOSE_HOARD: rps = closeHoardPolynomial.predict(distMeters);  break;
                case FAR_HOARD:   rps = farHoardPolynomial.predict(distMeters);    break;
                case OPP_HOARD:   rps = oppHoardPolynomial.predict(distMeters);    break;
            }
        } else {
            switch (mode) {
                case HUB:         rps = hubFlywheelMap.get(distMeters); break;
                case CLOSE_HOARD: rps = closeHoardFlywheelMap.get(distMeters); break;
                case FAR_HOARD:   rps = farHoardFlywheelMap.get(distMeters); break;
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
            case CLOSE_HOARD:
                slope = CLOSE_HOARD_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_CLOSE_HOARD[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_CLOSE_HOARD[0];
                break;
            case FAR_HOARD:
                slope = FAR_HOARD_HOOD_SLOPE;
                yInterceptAngle = EpsilonRegression.CLOSEST_HOOD_ANGLE_FAR_HOARD[1];
                xInterceptDist  = EpsilonRegression.CLOSEST_HOOD_ANGLE_FAR_HOARD[0];
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
