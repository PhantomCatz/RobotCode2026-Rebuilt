package frc.robot.CatzSubsystems.CatzClimb;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.Util;

public class ClimbConstants {
	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(Units.Inches.of(1.0));

	public static final Distance FULL_EXTENSION = Units.Inches.of(12.0);
	public static final Setpoint FULL_EXTEND = Setpoint.withMotionMagicSetpoint(converter.toAngle(FULL_EXTENSION));
	public static final Distance home = Units.Inches.of(0.0);
	public static final Setpoint HOME = Setpoint.withMotionMagicSetpoint(converter.toAngle(home));

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN2 -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

	public static final Setpoint Stow = Setpoint.withMotionMagicSetpoint(0.0);
	public static final Setpoint Extend = Setpoint.withMotionMagicSetpoint(50.0);

    private static final int CLIMB_MOTOR_ID = 24;

	public static final Distance CLIMB_THRESHOLD = Units.Inches.of(1.0);

    public static final Setpoint OFF = Setpoint.withVelocitySetpoint(0.0);

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 50.0;


		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 12.0; //TODO dont use magic number

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = CLIMB_MOTOR_ID; //TODO magic numbers!!
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
