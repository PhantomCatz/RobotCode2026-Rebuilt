package frc.robot.CatzSubsystems.CatzArm.CatzMantaPivot;

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

public class MantaPivotConstants {
	// TODO make real numbers
	// Arm angle in rotations
	public static final Angle STOW_POSITION = Units.Rotations.of(0.0);
	public static final Angle HORIZONTAL_POSITION = Units.Rotation.of(0.0);
	public static final Angle HOLD_POSITION = Units.Rotations.of(0.20);

	public static final Angle L1_FRONT_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.05));
	public static final Angle L2_FRONT_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.10));
	public static final Angle L3_FRONT_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.15));
	public static final Angle L4_FRONT_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.20));

	public static final Angle L1_BACK_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.45));
	public static final Angle L2_BACK_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.40));
	public static final Angle L3_BACK_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.35));
	public static final Angle L4_BACK_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.30));

	public static final Angle ALGAE_FRONT_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.15));
	public static final Angle ALGAE_BACK_POSITION = HORIZONTAL_POSITION.plus(Units.Rotations.of(0.35));

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

	public static final Setpoint[][] PIVOT_SCORE_SETPOINTS = {{L1_FRONT, L2_FRONT, L3_FRONT, L4_FRONT}, {L1_BACK, L2_BACK, L3_BACK, L4_BACK}};
	public static final Setpoint[] PIVOT_ALGAE_SETPOINTS = {ALGAE_FRONT, ALGAE_BACK};

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN_MANTA -> new Gains(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };


    public static final LoggedTunableNumber kP = new LoggedTunableNumber("MantaPivot Deploy/kP", gains.kP());
	public static final LoggedTunableNumber kI = new LoggedTunableNumber("MantaPivot Deploy/kI", gains.kI());
	public static final LoggedTunableNumber kD = new LoggedTunableNumber("MantaPivot Deploy/kD", gains.kD());
	public static final LoggedTunableNumber kS = new LoggedTunableNumber("MantaPivot Deploy/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("MantaPivot Deploy/kV", gains.kV());
	public static final LoggedTunableNumber kA = new LoggedTunableNumber("MantaPivot Deploy/kA", gains.kA());
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("MantaPivot Deploy/kG", gains.kG());

    // public static final LoggedTunableNumber kD = new LoggedTunableNumber("MantaPivot Deploy/kD", gains.kD());
    // public static final LoggedTunableNumber kS = new LoggedTunableNumber("MantaPivot Deploy/kS", gains.kS());

	private static final int MANTA_PIVOT_DEPLOY_MOTOR_ID = 11;

	public static final Angle DEPLOY_THRESHOLD = Units.Degrees.of(2.0);
	public static final double GRAVITY_FEEDFORWARD = 0.7;
	// public static final LoggedTunableNumber kG = new LoggedTunableNumber("MantaPivot Deploy/kG", GRAVITY_FEEDFORWARD);

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

		FXConfig.Feedback.SensorToMechanismRatio = 60/11*46/30*56/18*54/12;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = MANTA_PIVOT_DEPLOY_MOTOR_ID;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {MotorAlignmentValue.Aligned, MotorAlignmentValue.Opposed, MotorAlignmentValue.Opposed};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {12,13,14};
		return IOConfig;
	}
}
