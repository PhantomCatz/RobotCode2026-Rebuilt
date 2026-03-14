package frc.robot.CatzSubsystems.CatzBoba;

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
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class BobaConstants {

	public static final Angle BOBA_THRESHOLD = Units.Degrees.of(2.0);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.0, 0, 0.0, 0.0, 0.0, 0, 0.0);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		case BUBBLES -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		case SN_TEST -> new Gains(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
		default -> new Gains(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Boba/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Boba/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Boba/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Boba/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Boba/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Boba/kA", gains.kA());

    private static final int BOBA_MOTOR_ID = 20;


    public static final Setpoint OFF = Setpoint.withVoltageSetpoint(0.0);
	public static final Setpoint FAST = Setpoint.withVoltageSetpoint(3.0);
	public static final Setpoint POS = Setpoint.withMotionMagicSetpoint(10.0);

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 28.75;
        FXConfig.MotionMagic.MotionMagicAcceleration = 64.3;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 300.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;



		FXConfig.Feedback.SensorToMechanismRatio = 1;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = BOBA_MOTOR_ID; //TODO magic numbers!!
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {}; //TODO magic numbers!!
		return IOConfig;
	}
}
