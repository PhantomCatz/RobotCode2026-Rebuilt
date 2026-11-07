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
import frc.robot.CatzSubsystems.CatzArm.CatzElevator.CatzElevator.ArmState;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class ElevatorConstants {
	// TODO make real numbers
	// inches of arm (even though it says rotations)
	public static final Angle STOW_POSITION = Units.Rotations.of(0.0);
	public static final Angle HOLD_POSITION = Units.Rotations.of(0.20); //0.388 max

	public static final Angle L1_FRONT_POSITION = Units.Rotations.of(5.0);
	public static final Angle L2_FRONT_POSITION = Units.Rotations.of(10.0);
	public static final Angle L3_FRONT_POSITION = Units.Rotations.of(15.0);
	public static final Angle L4_FRONT_POSITION = Units.Rotations.of(20.0);

	public static final Angle L1_BACK_POSITION = Units.Rotations.of(5.0);
	public static final Angle L2_BACK_POSITION = Units.Rotations.of(10.0);
	public static final Angle L3_BACK_POSITION = Units.Rotations.of(15.0);
	public static final Angle L4_BACK_POSITION = Units.Rotations.of(20.0);

	public static final Angle ALGAE_FRONT_POSITION = Units.Rotations.of(25.0);
	public static final Angle ALGAE_BACK_POSITION = Units.Rotations.of(25.0);

	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(STOW_POSITION);
	public static final Setpoint HOLD = Setpoint.withMotionMagicSetpoint(HOLD_POSITION);

	public static final Setpoint L1_FRONT = Setpoint.withMotionMagicSetpoint(L1_FRONT_POSITION);
	public static final Setpoint L2_FRONT = Setpoint.withMotionMagicSetpoint(L2_FRONT_POSITION);
	public static final Setpoint L3_FRONT = Setpoint.withMotionMagicSetpoint(L3_FRONT_POSITION);
	public static final Setpoint L4_FRONT = Setpoint.withMotionMagicSetpoint(L4_FRONT_POSITION);

	public static final Setpoint L1_BACK = Setpoint.withMotionMagicSetpoint(L1_BACK_POSITION);
	public static final Setpoint L2_BACK = Setpoint.withMotionMagicSetpoint(L2_BACK_POSITION);
	public static final Setpoint L3_BACK = Setpoint.withMotionMagicSetpoint(L3_BACK_POSITION);
	public static final Setpoint L4_BACK = Setpoint.withMotionMagicSetpoint(L4_BACK_POSITION);

	public static final Setpoint ALGAE_FRONT = Setpoint.withMotionMagicSetpoint(ALGAE_FRONT_POSITION);
	public static final Setpoint ALGAE_BACK = Setpoint.withMotionMagicSetpoint(ALGAE_BACK_POSITION);

	public static final Setpoint[][] ELEVATOR_SCORE_SETPOINTS = {{L1_FRONT, L2_FRONT, L3_FRONT, L4_FRONT}, {L1_BACK, L2_BACK, L3_BACK, L4_BACK}};
	public static final Setpoint[] ELEVATOR_ALGAE_SETPOINTS = {ALGAE_FRONT, ALGAE_BACK};
	public static final ArmState[][] ARM_SCORE_STATES = {{ArmState.L1_FRONT, ArmState.L2_FRONT, ArmState.L3_FRONT, ArmState.L4_FRONT}, {ArmState.L1_BACK, ArmState.L2_BACK, ArmState.L3_BACK, ArmState.L4_BACK}};
	public static final ArmState[] ARM_ALGAE_STATES = {ArmState.ALGAE_FRONT, ArmState.ALGAE_BACK};

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

		FXConfig.Feedback.SensorToMechanismRatio = 4*1.432*Math.PI*2; // rotations of motor for one inch raise elevator

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
