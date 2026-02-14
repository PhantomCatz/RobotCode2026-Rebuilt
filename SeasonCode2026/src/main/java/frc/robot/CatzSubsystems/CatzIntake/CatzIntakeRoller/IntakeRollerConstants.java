package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeRollerConstants {

	public static final Setpoint OFF_SETPOINT = Setpoint.withVoltageSetpoint(0.0);
	public static final Setpoint ON_SETPOINT = Setpoint.withVoltageSetpoint(10.0);
	//public static final Setpoint ON_SETPOINT = Setpoint.withVoltageSetpoint(5.0);
	public static final Setpoint S_SETPOINT = Setpoint.withDutyCycleSetpoint(0.7);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.0, 0, 0.0, 0.0, 0.0, 0, 0.0);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

	private static final int INTAKE_MOTOR_ID = 15;

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 50.0;


		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 60.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 0.0; //TODO dont use magic number

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = INTAKE_MOTOR_ID;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {};
		return IOConfig;
	}
}
