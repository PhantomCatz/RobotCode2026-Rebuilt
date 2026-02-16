package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {

    //------Hub Scoring--------
    public static final double[] CLOSEST_HOOD_ANGLE_HUB = {1.294, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE_HUB = {5.350, 42.0};

    public static double[][] flywheelHubRPS = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        {1.294, 30.0},
        {2.71, 34.0},
        {4.18, 38.0},
        {5.350, 42.0}
    };

    public static double[][] airtimeHub = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> airtime of the ball (in seconds)
        { 5.350, 1.20 },
        {6.7, 6.7},
        {6.9, 6.9}

    };


    //-------Closest Corner Hoarding----------
    public static final double[] CLOSEST_HOOD_ANGLE_CLOSE_HOARD = {1.294, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE_CLOSE_HOARD = {5.350, 42.0};

    public static double[][] flywheelCloseHoardRPS = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        {1.294, 30.0},
        {2.71, 34.0},
        {4.18, 38.0},
        {5.350, 42.0}
    };


    //-------Far Corner Hoarding--------------
    public static final double[] CLOSEST_HOOD_ANGLE_FAR_HOARD = {1.294, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE_FAR_HOARD = {5.350, 42.0};

    public static double[][] flywheelFarHoardRPS = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        {1.294, 30.0},
        {2.71, 34.0},
        {4.18, 38.0},
        {5.350, 42.0}
    };

    //-------Opposite Alliance Hoarding-------

    public static final double[] CLOSEST_HOOD_ANGLE_OPP_HOARD = {1.294, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE_OPP_HOARD = {5.350, 42.0};

    public static double[][] flywheelOppHoardRPS = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        {1.294, 30.0},
        {2.71, 34.0},
        {4.18, 38.0},
        {5.350, 42.0}
    };
}
