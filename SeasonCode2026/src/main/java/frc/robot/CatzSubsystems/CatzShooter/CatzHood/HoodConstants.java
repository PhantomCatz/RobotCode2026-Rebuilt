package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.MotorUtil.Gains;

public class HoodConstants {
	public static final Angle HOOD_ZERO_POS = Units.Degrees.of(16.0);
	public static final Angle HOOD_MAX_POS = Units.Degrees.of(45.0);
	public static final Angle HOOD_TEST_POS = Units.Degrees.of(35.0);
	public static final Setpoint HOOD_STOW_SETPOINT = Setpoint.withMotionMagicSetpoint(HOOD_ZERO_POS);
	public static final Setpoint HOOD_TEST_SETPOINT = Setpoint.withMotionMagicSetpoint(HOOD_TEST_POS);
	public static final Setpoint HOOD_STOP = Setpoint.withVelocitySetpoint(0.0);

	// Set home position constants
	public static final Setpoint HOOD_HOME_SETPOINT = Setpoint.withVelocitySetpoint(-0.3);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(35.0, 0.0, 3.0, 0.25, 1.4,0.0, 0.2);
        case SN2 -> new Gains(35.0, 0.0, 3.0, 0.25, 1.4,0.0, 0.2);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA", gains.kA());
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", gains.kG());

	public static final LoggedTunableNumber adjustableHoodAngle = new LoggedTunableNumber("Hood/HoodAngle", HOOD_ZERO_POS.in(Units.Degrees));

    private static final int HOOD_MOTOR_ID = 22;

	public static final Angle HOOD_THRESHOLD = Units.Degrees.of(1.0);

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 500.0;
		FXConfig.MotionMagic.MotionMagicJerk = 4000.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 40.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_MAX_POS.in(Units.Rotations);
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD_ZERO_POS.in(Units.Rotations);

		FXConfig.Feedback.SensorToMechanismRatio = 184 / 10.0; //10.0 / 184.0 / 0.015267 * 5514.2857; //TODO dont use magic number
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = HOOD_MOTOR_ID;
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
