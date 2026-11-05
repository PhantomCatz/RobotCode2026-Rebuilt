package frc.robot.CatzSubsystems.CatzArm.CatzElevator;

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
	// TODO make real numbers
	public static final Angle STOW_POSITION = Units.Rotations.of(0.0);
	public static final Angle HOLD_POSITION = Units.Rotations.of(0.20); //0.388 max
	public static final Angle L1_POSITION = Units.Rotations.of(0.20);
	public static final Angle L2_POSITION = Units.Rotations.of(0.20);
	public static final Angle L3_POSITION = Units.Rotations.of(0.20);
	public static final Angle L4_POSITION = Units.Rotations.of(0.20);
	public static final Angle ALGAE_POSITION = Units.Rotations.of(0.20);

	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(STOW_POSITION);
	public static final Setpoint HOLD = Setpoint.withMotionMagicSetpoint(HOLD_POSITION);
	public static final Setpoint L1 = Setpoint.withMotionMagicSetpoint(L1_POSITION);
	public static final Setpoint L2 = Setpoint.withMotionMagicSetpoint(L2_POSITION);
	public static final Setpoint L3 = Setpoint.withMotionMagicSetpoint(L3_POSITION);
	public static final Setpoint L4 = Setpoint.withMotionMagicSetpoint(L4_POSITION);
	public static final Setpoint ALGAE = Setpoint.withMotionMagicSetpoint(ALGAE_POSITION);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
		case SN_MANTA -> new Gains(5.5, 0, 0.0, 0.35, 0.0, 0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
	public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", gains.kI());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", gains.kV());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", gains.kS());
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", gains.kG());
	public static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", gains.kA());

	private static final int ELEVATOR_MOTOR_ID = 30;

	public static final Angle ELEVATOR_THRESHOLD = Units.Degrees.of(2.0);
	public static final double GRAVITY_FEEDFORWARD = 0.7;

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kI = gains.kI();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kA = gains.kA();
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

		FXConfig.Feedback.SensorToMechanismRatio = 4*1.432*Math.PI*2; // inches the elevator raises per rotation of motor

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


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
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {MotorAlignmentValue.Aligned};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {31};
		return IOConfig;
	}
}
