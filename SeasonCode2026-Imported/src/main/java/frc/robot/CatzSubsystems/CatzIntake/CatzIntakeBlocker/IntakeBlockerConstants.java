package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeBlocker;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class IntakeBlockerConstants {
	public static final Angle HOME_POSITION = Units.Degrees.of(0.0);
	public static final Angle BLOCKER_POSITION = Units.Rotations.of(-0.04);

	private static final int INTAKE_BLOCKER_BUS_ID = 2;
	private static final int INTAKE_BLOCKER_MOTOR_ID = 32;
	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(HOME_POSITION);
	public static final Setpoint BLOCKER = Setpoint.withMotionMagicSetpoint(BLOCKER_POSITION);
	public static final Angle BLOCKER_THRESHOLD = Units.Degrees.of(2.0);
	public static final double GRAVITY_FEEDFORWARD = 2 ; // CHANGE !!!!

	public static final Gains gains = switch (CatzConstants.getRobotType()) {
		case SN1 -> new Gains(0.5, 0, 0.0, 0.35, 0.0, 0, 1.9);
		case SN2 -> new Gains(30.0, 1.0, 2.0, 0.0, 2, 0.0, 0.0);
		case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	};

	public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake Blocker/kP", gains.kP());
	public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake Blocker/kV", gains.kV());

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 0.85;
		FXConfig.MotionMagic.MotionMagicAcceleration = 25.0;
		FXConfig.MotionMagic.MotionMagicJerk = 250.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.Feedback.SensorToMechanismRatio = 14.95;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = INTAKE_BLOCKER_MOTOR_ID;
		IOConfig.mainBus = INTAKE_BLOCKER_BUS_ID;
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {};
		IOConfig.followerBuses = new int[] { 2, 2 };
		IOConfig.followerIDs = new int[] {};
		return IOConfig;
	}
}
