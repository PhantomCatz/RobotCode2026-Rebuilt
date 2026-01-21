package frc.robot.CatzSubsystems.CatzHood;

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
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.MotorUtil.Gains;

public class HoodConstants {
	public static final Angle HOOD_ZERO_POS = Units.Degrees.of(30.0);
	public static final Angle HOOD_MAX_POS = Units.Degrees.of(60.0);
	public static final Angle HOOD_TEST_POS = Units.Degrees.of(50.0);
	public static final Setpoint HOOD_STOW_SETPOINT = Setpoint.withMotionMagicSetpoint(HOOD_ZERO_POS);
	public static final Setpoint HOOD_TEST_SETPOINT = Setpoint.withMotionMagicSetpoint(HOOD_TEST_POS);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0, 0.0);
        case SN2 -> new Gains(70.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", gains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA", gains.kA());
	public static final LoggedTunableNumber adjustableHoodAngle = new LoggedTunableNumber("Hood/HoodAngle", 0.0);

    private static final int HOOD_MOTOR_ID = 29;

	public static final Angle HOOD_THRESHOLD = Angle.ofBaseUnits(1.0, Units.Degrees);


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
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 184 / 10.0; //10.0 / 184.0 / 0.015267 * 5514.2857; //TODO dont use magic number

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
