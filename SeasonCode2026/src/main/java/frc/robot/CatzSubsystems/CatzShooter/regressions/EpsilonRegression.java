package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {
    public static final double[] CLOSEST_HOOD_ANGLE = {1.294, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE = {4.659, 42.0};

    public static double[][] flywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        { 1.294, 30 },
        {2.316, 34},
        {3.269, 37},
        {4.897, 40}
        // {4.659, }

    };

    public static double[][] airtime = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in inches)
        // @y --> shooter velocity (in rps)
        { 53.0, 20 }

    };
}
