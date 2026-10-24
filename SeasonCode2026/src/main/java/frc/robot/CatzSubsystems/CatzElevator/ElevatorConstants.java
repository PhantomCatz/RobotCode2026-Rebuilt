package frc.robot.CatzSubsystems.CatzElevator;

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

public class ElevatorConstants {
	public static final Angle STOW_POSITION = Units.Rotations.of(0.0);
	public static final Angle DEPLOY_POSITION = Units.Rotations.of(0.30);

	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(STOW_POSITION);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(DEPLOY_POSITION);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
		case SN_MANTA -> new Gains(0.5, 0, 0.0, 0.35, 0.0, 0, 1.9);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", gains.kV());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", gains.kS());
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", gains.kG());

	private static final int ELEVATOR_MOTOR_ID = 1000000; // TODO fix

	public static final Angle ELEVATOR_THRESHOLD = Units.Degrees.of(2.0);
	public static final double GRAVITY_FEEDFORWARD = 0.7;

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
		IOConfig.mainID = ELEVATOR_MOTOR_ID;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {100000}; // TODO fix
		return IOConfig;
	}
}
