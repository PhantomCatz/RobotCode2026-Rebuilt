package frc.robot.CatzSubsystems.CatzArm;

import edu.wpi.first.math.geometry.Translation3d;

public class ArmConstants {
    public static boolean isArmDisabled = false;
    // ALL TODO
    public static double ARM_GEAR_REDUCTION = 1;
    public static double ARM_JKG_SQUARED = 0.025;
    public static double ARM_LENGTH_INCHES = 30;
    public static double ARM_MIN_DEGREES = 0;
    public static double ARM_MAX_DEGREES = 180;
    public static double ARM_INITIAL_DEGREES = 0;

    public static Translation3d ARM_SIM_OFFSET = new Translation3d(0, 0, 0);
}
