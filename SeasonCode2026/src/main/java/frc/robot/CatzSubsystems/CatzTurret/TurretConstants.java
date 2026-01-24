package frc.robot.CatzSubsystems.CatzTurret;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class TurretConstants {
	public static final Setpoint HOME_SETPOINT = Setpoint.withPositionSetpoint(Units.Degrees.of(0.0));

	public static final Setpoint no = Setpoint.withMotionMagicSetpoint(0);
	public static final Setpoint alittle = Setpoint.withMotionMagicSetpoint(50);
	public static final Setpoint alittelbackwrads = Setpoint.withMotionMagicSetpoint(-50);
	public static final Setpoint WEEEEEE = Setpoint.withMotionMagicSetpoint(135);
	public static final Setpoint backwordsbeastmode = Setpoint.withMotionMagicSetpoint(-135);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0, 0.0);
        case SN_MANTA -> new Gains(100.0, 0.0, 0.00, 0.22, 4.8, 0.0, 0.0); // kd 0.05
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

	public static final Transform2d TURRET_OFFSET = new Transform2d( edu.wpi.first.math.util.Units.inchesToMeters(-4.0),  edu.wpi.first.math.util.Units.inchesToMeters(9.5), new Rotation2d()){

	};

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", gains.kA());

	public static final Angle HOME_POSITION = Units.Degrees.of(180.0);
    private static final int TURRET_MOTOR_ID = 12;

	public static final Angle TURRET_THRESHOLD = Units.Degrees.of(1.0);

	public static final Angle TURRET_MAX = Units.Degrees.of(180);
	public static final Angle TURRET_MIN = Units.Degrees.of(-180);


	public static final int NUM_OF_FULL_ROT = 1;

	public static final double ROBOT_OMEGA_FEEDFORWARD = 1.0;//25;
	public static final double ROBOT_ACCELERATION_FEEDFORWARD = 0.00;

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kI = gains.kI();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
		FXConfig.Slot0.kA = gains.kA();
		FXConfig.Slot0.kG = gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0/42.5;//100.0 / 42.0;
        FXConfig.MotionMagic.MotionMagicAcceleration = 10.0;
		FXConfig.MotionMagic.MotionMagicJerk = 100.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 42.5;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = TURRET_MOTOR_ID;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
						// .withForwardSoftLimitThreshold(TURRET_MAX)
						// .withReverseSoftLimitThreshold(TURRET_MIN)); //NOTE add back soft limits
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {};
		return IOConfig;
	}
}
