package frc.robot.CatzSubsystems.CatzClimbElevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.Util;

public class ClimbConstants {
	private static final double SPOOL_DIAMETER_INCH = 0.95;
	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(Units.Inches.of(SPOOL_DIAMETER_INCH / 2.0));

	private static final Angle REACH_POSITION = converter.toAngle(Units.Inches.of(5.0));
	private static final Angle STOW_POSITION = converter.toAngle(Units.Inches.of(0.0));

	public static final Setpoint REACH_SETPOINT = Setpoint.withMotionMagicSetpoint(REACH_POSITION.in(Units.Rotations));
	public static final Setpoint STOW_SETPOINT = Setpoint.withMotionMagicSetpoint(STOW_POSITION.in(Units.Rotations));

	public static final Distance FULL_EXTENSION = Units.Inches.of(12.0);
	public static final Setpoint FULL_EXTEND = Setpoint.withMotionMagicSetpoint(converter.toAngle(FULL_EXTENSION));
	public static final Distance home = Units.Inches.of(0.0);
	public static final Setpoint HOME = Setpoint.withMotionMagicSetpoint(converter.toAngle(home));

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    // private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
    // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
    // private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
    // private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
    // private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
    // private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

    private static final int CLIMB_MOTOR_ID_1 = 60;
	private static final int CLIMB_MOTOR_ID_2 = 61;

	public static final Distance CLIMB_THRESHOLD = Units.Inches.of(1.0);
	public static final double CLAW_EXTEND_THRESHOLD = 10.0; //TODO add real number
	public static final double CLAW_RETRACT_THRESHOLD = 10.0; //TODO add real number

    public static final Setpoint OFF = Setpoint.withVoltageSetpoint(0.0);

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 28.75 / (2*Math.PI*SPOOL_DIAMETER_INCH/2.0);
        FXConfig.MotionMagic.MotionMagicAcceleration = 64.3 / (2*Math.PI*SPOOL_DIAMETER_INCH/2.0);

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 80.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 19.25; //TODO dont use magic number

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = CLIMB_MOTOR_ID_1; //TODO magic numbers!!
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {MotorAlignmentValue.Aligned};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {CLIMB_MOTOR_ID_2}; //TODO magic numbers!!
		return IOConfig;
	}
}
