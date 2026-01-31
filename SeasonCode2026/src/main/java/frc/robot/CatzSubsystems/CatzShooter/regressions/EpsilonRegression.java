package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {
    public static final double[] CLOSEST_HOOD_ANGLE = {53.0, 30.0}; //distance from target (meters), hood angle (degrees)
    public static final double[] FARTHEST_HOOD_ANGLE = {199.0, 28.0};

    public static double[][] flywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in inches)
        // @y --> shooter velocity (in rps)
        { 53.0, 20 },
        {199.0, 42}

    };

    public static double[][] airtime = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in inches)
        // @y --> shooter velocity (in rps)
        { 53.0, 20 }

    };
}
