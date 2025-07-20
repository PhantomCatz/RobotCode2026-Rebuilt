package frc.robot.CatzSubsystems.CatzArm;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class ArmConstants {
    public static boolean isArmDisabled = false;
    // ALL TODO

    public static int ARM_MOTOR_ID = 0;
    public static final double CURRENT_LIMIT = 40.0;

    public static double ARM_GEAR_REDUCTION = 1;
    public static double ARM_JKG_SQUARED = 0.025;
    public static double ARM_LENGTH_INCHES = 30;
    public static double ARM_MIN_DEGREES = 0;
    public static double ARM_MAX_DEGREES = 180;
    public static double ARM_INITIAL_DEGREES = 0;

    public static Translation3d ARM_SIM_OFFSET = new Translation3d(0, 0, 0);

    // Initial PIDF and motion magic assignment
    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            case SN1 -> new Gains(10.0, 0.0, 0.1, 0.00, 0.0, 0.0, 3.0); //kg1.5
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final Gains slot1_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(2.0, 0.0, 0.0, 0.175, 0.130, 0.013, 0.4);
            case SN1 -> new Gains(1.0, 0.1, 0.0, 0.175, 0.13, 0.013, 0.4); //
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(100.0, 200.0, 800.0);
            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    public static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("Arm/tunnable Pos", 1);
    public static LoggedTunableNumber zeroPos  = new LoggedTunableNumber("Arm/ZeroPos", 0.0);

    public static LoggedTunableNumber slot0_kP = new LoggedTunableNumber("Arm/slot_0 kP", 4.0);
    public static LoggedTunableNumber slot0_kI = new LoggedTunableNumber("Arm/slot_0 kI", 0.0);
    public static LoggedTunableNumber slot0_kD = new LoggedTunableNumber("Arm/slot_0 kD", 0.0);
    public static LoggedTunableNumber slot0_kS = new LoggedTunableNumber("Arm/slot_0 kS", 0.0);
    public static LoggedTunableNumber slot0_kV = new LoggedTunableNumber("Arm/slot_0 kV", 0.0);
    public static LoggedTunableNumber slot0_kA = new LoggedTunableNumber("Arm/slot_0 kA", 0.0);

    public static LoggedTunableNumber slot1_kP = new LoggedTunableNumber("Arm/slot_1 kP", 4.0);
    public static LoggedTunableNumber slot1_kI = new LoggedTunableNumber("Arm/slot_1 kI", 0.0);
    public static LoggedTunableNumber slot1_kD = new LoggedTunableNumber("Arm/slot_1 kD", 0.0);
    public static LoggedTunableNumber slot1_kS = new LoggedTunableNumber("Arm/slot_1 kS", 0.0);
    public static LoggedTunableNumber slot1_kV = new LoggedTunableNumber("Arm/slot_1 kV", 0.0);
    public static LoggedTunableNumber slot1_kA = new LoggedTunableNumber("Arm/slot_1 kA", 0.0);
}
