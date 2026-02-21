package frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw;


import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.io.GenericSparkmaxIOReal.MotorIOSparkMaxConfig;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.Util;

public class ClimbConstantsClaw {
	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(Units.Inches.of(1.0));

	public static final Setpoint FULL_EXTEND = Setpoint.withVoltageSetpoint(0.5);
	public static final Setpoint HOME = Setpoint.withMotionMagicSetpoint(1);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };


    private static final int CLIMB_MOTOR_ID = 5;

	public static final Distance CLIMB_THRESHOLD = Units.Inches.of(1.0);

    public static final Setpoint OFF = Setpoint.withVelocitySetpoint(0.0);

    // public static final TalonFXConfiguration getFXConfig() {
	// 	TalonFXConfiguration FXConfig = new TalonFXConfiguration();
	// 	FXConfig.Slot0.kP = gains.kP();
	// 	FXConfig.Slot0.kD = gains.kD();
	// 	FXConfig.Slot0.kS = gains.kS();
	// 	FXConfig.Slot0.kG = gains.kG();

	// 	FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
    //     FXConfig.MotionMagic.MotionMagicAcceleration = 50.0;


	// 	FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
	// 	FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
	// 	FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
	// 	FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

	// 	FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
	// 	FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

	// 	FXConfig.Voltage.PeakForwardVoltage = 12.0;
	// 	FXConfig.Voltage.PeakReverseVoltage = -12.0;


	// 	FXConfig.Feedback.SensorToMechanismRatio = 1.0; //TODO dont use magic number

	// 	FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

	// 	return FXConfig;
	// }

	public static  MotorIOSparkMaxConfig getIOConfig() {
		MotorIOSparkMaxConfig IOConfig = new  MotorIOSparkMaxConfig();


		IOConfig.mainID = CLIMB_MOTOR_ID; //TODO magic numbers!!

		IOConfig.gearRatio = 1;
		SparkMaxConfig config = new SparkMaxConfig();

		return IOConfig;
	}

	// public static class MotorIOSparkMaxConfig {
    //     public int mainID = -1;
    //     public int[] followerIDs = new int[0];
    //     public boolean[] followerOpposeMain = new boolean[0];

    //     public boolean invertMotor = false;
    //     public int currentLimitAmps = 40;
    //     public double gearRatio = 1.0;
    // }
}
