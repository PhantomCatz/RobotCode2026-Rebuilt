package frc.robot.CatzSubsystems.CatzShooter.regressions;

import frc.robot.Utilities.InterpolatingDouble;
import frc.robot.Utilities.InterpolatingTreeMap;
import frc.robot.Utilities.PolynomialRegression;

public class ShooterRegression {
    public static final double kHoodPaddingDegrees = 2;
    public static final double kShooterPaddingVelocity = 100;


    public static final double[] kPadding = {
            kShooterPaddingVelocity, kHoodPaddingDegrees};

    //hood
    public static double kDefaultHoodAngle = Math.toRadians(0);
    public static boolean kUseHoodAutoAimPolynomial = false;

    public static boolean kUseSmartdashboard = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kHoodAutoAimPolynomial;

    public static double[][] kHoodRegression;

    static {

        kHoodRegression = EpsilonRegression.kHoodManualAngle;
        
        //iterate through the array and place each point into the interpolating tree
        for (double[] pair : kHoodRegression) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        
        kHoodAutoAimPolynomial = new PolynomialRegression(kHoodRegression, 1);
    }
    
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double[][] kFlywheelRegression;

    static {

        kFlywheelRegression = EpsilonRegression.kFlywheelManualRPM;
        
        for (double[] pair : kFlywheelRegression) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelRegression, 2);
    }

    private static final double HOOD_ANGLE_SLOPE = (EpsilonRegression.FARTHEST_HOOD_ANGLE[1]-EpsilonRegression.CLOSEST_HOOD_ANGLE[1]) / 
                                                   (EpsilonRegression.FARTHEST_HOOD_ANGLE[0]-EpsilonRegression.CLOSEST_HOOD_ANGLE[0]);

    public static double getHoodAngle(double distance){
        return HOOD_ANGLE_SLOPE * (distance - EpsilonRegression.CLOSEST_HOOD_ANGLE[0]) + EpsilonRegression.CLOSEST_HOOD_ANGLE[1];
    }

}
