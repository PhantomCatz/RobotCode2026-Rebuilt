package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IntakeDeployConstants {
	public static final Angle HOME_POSITION = Units.Degrees.of(0.0);
	public static final Angle DEPLOY_POSITION = Units.Degrees.of(90.0);

	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(HOME_POSITION);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(DEPLOY_POSITION);

	public static final Setpoint HoldDown = Setpoint.withVoltageSetpoint(6.0);
	public static final Setpoint Zero = Setpoint.withVoltageSetpoint(0.0);
	public static final Setpoint Sixty = Setpoint.withMotionMagicSetpoint(Units.Degrees.of(60));

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.5, 0, 0.0, 0.35, 0.0, 0, 1.9);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake Deploy/kP", gains.kP());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake Deploy/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake Deploy/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake Deploy/kV", gains.kV());

	private static final int INTAKE_DEPLOY_MOTOR_ID = 30;

	public static final Angle DEPLOY_THRESHOLD = Units.Degrees.of(2.0);
	public static final double GRAVITY_FEEDFORWARD = 2.5;
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake Deploy/kG", GRAVITY_FEEDFORWARD);

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 5.0;
		FXConfig.MotionMagic.MotionMagicJerk = 10.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 60.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.Feedback.SensorToMechanismRatio = 14.95;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = INTAKE_DEPLOY_MOTOR_ID;
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
