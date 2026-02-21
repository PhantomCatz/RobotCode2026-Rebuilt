package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {

    // Hub Scoring
    public static final double[] CLOSEST_HOOD_ANGLE_HUB = {1.294, 16.0}; 
    public static final double[] FARTHEST_HOOD_ANGLE_HUB = {5.350, 42.0};

    public static double[][] flywheelHubRPS = {
        {1.294, 32.0},
        {2.71, 36.0},
        {4.18, 40.0},
        {5.350, 44.0}
    };

    public static double[][] airtimeHub = {
        {4.638, 1.18},
        {3.154, 1.20},
        {2.134, 1.12}
    };


    // Closest Corner Hoarding
    public static final double[] CLOSEST_HOOD_ANGLE_OVER_TRENCH_HOARD = {5.029, 45.0}; 
    public static final double[] FARTHEST_HOOD_ANGLE_OVER_TRENCH_HOARD = {9.314, 45.0};

    public static double[][] flywheelOverTrenchHoardRPS = {
        {5.768, 30.0},
        {7.528, 40.0},
        {9.314, 50.0}
    };

    public static double[][] airtimeOverTrenchHoard = {
        {9.314, 1.50},
        {7.528, 1.30},
        {5.768, 1.10}
    };


    // Far Corner Hoarding
    public static final double[] CLOSEST_HOOD_ANGLE_OVER_NET_HOARD = {6.959, 17.0}; 
    public static final double[] FARTHEST_HOOD_ANGLE_OVER_NET_HOARD = {10.457, 30.0};

    public static double[][] flywheelOverNetHoardRPS = {
        {6.959, 60.0},
        {8.5, 62.5},
        {10.457, 65.0}
    };

    public static double[][] airtimeOverNetHoard = {
        {10.457, 1.60},
        {8.500, 1.40},
        {6.959, 1.20}
    };

    // Opposite Alliance Hoarding

    public static final double[] CLOSEST_HOOD_ANGLE_OPP_HOARD = {1.294, 16.0}; 
    public static final double[] FARTHEST_HOOD_ANGLE_OPP_HOARD = {5.350, 42.0};

    public static double[][] flywheelOppHoardRPS = {
        {1.294, 30.0},
        {2.71, 34.0},
        {4.18, 38.0},
        {5.350, 42.0}
    };

    public static double[][] airtimeOppHoard = {
        {5.350, 1.30},
        {4.180, 1.20},
        {2.710, 1.00}
    };
}