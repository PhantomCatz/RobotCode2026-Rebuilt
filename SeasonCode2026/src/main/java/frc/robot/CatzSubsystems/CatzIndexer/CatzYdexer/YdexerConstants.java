package frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class YdexerConstants {
	private static final Voltage ON_VOLTS = Units.Volts.of(8.0);

	public static final Setpoint ON = Setpoint.withVoltageSetpoint(ON_VOLTS);
	public static final Setpoint OFF = Setpoint.withVoltageSetpoint(0.0);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0, 0.0);
        case SN2 -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

	public static final LoggedTunableNumber SPEED = new LoggedTunableNumber("YDexer/Applied Volts", ON_VOLTS.in(Units.Volts));

    private static final int YDEXER_MOTOR_ID = 50;

	private static final double[][] FLYWHEEL_VS_VOLTS = {
		//flywheel rps vs vdexer volts
		{60, }
	};

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 50.0;


		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 0.0; //TODO dont use magic number
		FXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = YDEXER_MOTOR_ID;
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
