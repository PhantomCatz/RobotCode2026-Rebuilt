package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {
    public static final double[] CLOSEST_HOOD_ANGLE = {1.288, 16.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE = {5.350, 42.0};

    public static double[][] flywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        {1.288, 0.0},
        {5.350, 42}
    };

    public static double[][] airtime = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rps)
        { 5.350, 1.20 }

    };
}
