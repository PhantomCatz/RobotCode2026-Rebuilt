package frc.robot.CatzSubsystems.CatzShooter.regressions;

public class EpsilonRegression {

        // Hub Scoring
        public static final double[] CLOSEST_HOOD_ANGLE_HUB = { 1.557, 22.0 }; // low
        // arc. static shooting
        public static final double[] FARTHEST_HOOD_ANGLE_HUB = { 5.549, 39.0 };

        public static final double HOOD_ANGLE_OFFSET = 0.0;

        public static double[][] flywheelHubRPS = {
                        // { 1.331, 28.5 }, // distance meters, flywheel rps
                        // { 2.915, 32.0 },
                        // { 4.780, 37.0 }
                        // { 1.246, 29.5 }, // distance meters, flywheel rps
                        // { 2.877, 32.5 },
                        // { 5.256, 39.0 }
                        { 1.279, 27.5 }, // distance meters, flywheel rps
                        { 3.167, 32.0 },
                        { 4.873, 37.5 }
        };

        public static double[][] airtimeHub = {
                        { 1.279, 0.975 }, // distance meters, time seconds
                        { 3.167, 0.95 },
                        { 4.873, 1.076 }
        };

        /*
         * Bisector Flywheel speed:
         *
         * 1.443
         *
         */
        // public static final double[] CLOSEST_HOOD_ANGLE_HUB = { 1.350, 14.0 }; //
        // high arc, for shoot while move.
        // public static final double[] FARTHEST_HOOD_ANGLE_HUB = { 5.192, 27.0 };

        // public static double[][] flywheelHubRPS = {
        // { 1.350, 30.0 },

        // { 5.192, 41.0 }
        // };

        // public static double[][] airtimeHub = {
        // { 5.549, 1.08 },
        // { 2.0202, 0.89 },
        // { 3.827, 1.03 }
        // };

        /*
         * High arc for shoot while move:
         *
         * 5.192, 41rps, 30deg
         * 1.350, 30rps, 17deg
         */

        // Closest Corner Hoarding
        public static final double[] CLOSEST_HOOD_ANGLE_OVER_TRENCH_HOARD = { 5.609, 30.0 };
        public static final double[] FARTHEST_HOOD_ANGLE_OVER_TRENCH_HOARD = {13.644, 41.0 }; // 6.952 hood 41 flywheel 35

        public static double[][] flywheelOverTrenchHoardRPS = {
                        { 5.609, 29.0 },
                        { 7.681, 37.0 },
                        { 13.644, 58.0 }
        };

        public static double[][] airtimeOverTrenchHoard = {
                        { 9.314, 1.50 },
                        { 7.528, 1.30 },
                        { 5.768, 1.10 }
        };

        // Far Corner Hoarding
        public static final double[] CLOSEST_HOOD_ANGLE_OVER_NET_HOARD = { 6.959, 17.0 };
        public static final double[] FARTHEST_HOOD_ANGLE_OVER_NET_HOARD = { 10.457, 30.0 };

        public static double[][] flywheelOverNetHoardRPS = {
                        { 6.959, 60.0 },
                        { 8.5, 62.5 },
                        { 10.457, 65.0 }
        };

        public static double[][] airtimeOverNetHoard = {
                        { 10.457, 1.60 },
                        { 8.500, 1.40 },
                        { 6.959, 1.20 }
        };

        // Opposite Alliance Hoarding

        public static final double[] CLOSEST_HOOD_ANGLE_OPP_HOARD = { 1.294, 16.0 };
        public static final double[] FARTHEST_HOOD_ANGLE_OPP_HOARD = { 5.350, 42.0 };

        public static double[][] flywheelOppHoardRPS = {
                        { 1.294, 30.0 },
                        { 2.71, 34.0 },
                        { 4.18, 38.0 },
                        { 5.350, 42.0 }
        };

        public static double[][] airtimeOppHoard = {
                        { 5.350, 1.30 },
                        { 4.180, 1.20 },
                        { 2.710, 1.00 }
        };
}
