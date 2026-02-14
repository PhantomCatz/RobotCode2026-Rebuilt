package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
		case SN1 -> new Gains(203.0, 0.0, 9.0, 0.4, 4.8, 0.0, 0.0);
        case SN_MANTA -> new Gains(100.0, 0.0, 0.00, 0.22, 4.8, 0.0, 0.0); // kd 0.05
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		case SN2 -> new Gains(203.0, 0.0, 9.0, 0.4, 4.8, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", gains.kA());

	public static final Angle HOME_POSITION = Units.Degrees.of(0.0);
    private static final int TURRET_MOTOR_ID = 25;

	public static final Angle TURRET_THRESHOLD = Units.Degrees.of(3.0);

	public static final Angle TURRET_MAX = Units.Degrees.of(120); // TODO change to 180 after turret is fixed
	public static final Angle TURRET_MIN = Units.Degrees.of(-120); // TODO change to 180 after turret is fixed

	public static final LoggedTunableNumber omegaFF = new LoggedTunableNumber("Turret/omegaFF", 5.0);
	public static final double ROBOT_OMEGA_FEEDFORWARD = 4.3;//25;
	public static final double ROBOT_ACCELERATION_FEEDFORWARD = 0.00;

	public static final Translation2d TURRET_OFFSET = new Translation2d(Units.Inches.of(-5).in(Units.Meters),  Units.Inches.of(5).in(Units.Meters));
	public static final Rotation2d TURRET_ROTATION_OFFSET = Rotation2d.fromDegrees(90.0);

	public static final CANcoder TURRET_CANCODER = new CANcoder(26);
public static final double CANCODER_RATIO = 1.0 / 8.5;//1.0 / 7.5;

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
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 50.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_MAX.in(Units.Rotations);
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_MIN.in(Units.Rotations);

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
