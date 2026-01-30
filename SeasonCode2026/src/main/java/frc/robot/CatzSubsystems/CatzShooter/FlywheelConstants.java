package frc.robot.CatzSubsystems.CatzShooter;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class FlywheelConstants {
	public static final Setpoint OFF_SETPOINT = Setpoint.withDutyCycleSetpoint(0.0);
	public static final Setpoint TEST_SETPOINT = Setpoint.withVelocitySetpoint(60.0);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0, 0.0);
        case SN2 -> new Gains(11.5
		, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
    // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
    // private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
    // private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
    // private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
    // private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());
    public static final LoggedTunableNumber SHOOTING_RPS_TUNABLE = new LoggedTunableNumber("Flywheels/EjectingRps", 60.0);

	public static final AngularVelocity FLYWHEEL_THRESHOLD = Units.RotationsPerSecond.of(5.0);
	public static final Translation2d VDEXER_FEED_COMPENSATION = new Translation2d(2.0, 0.0); //Because of the way we feed the balls into the shooter with the indexer, there is a directional bias in the ball trajectory.

    public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = gains.kP();
		FXConfig.Slot0.kD = gains.kD();
		FXConfig.Slot0.kS = gains.kS();
		FXConfig.Slot0.kV = gains.kV();
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


		FXConfig.Feedback.SensorToMechanismRatio = 1.0; //TODO dont use magic number

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = 30; //TODO magic numbers!!
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerAlignmentValue = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		IOConfig.followerBuses = new String[] {""};
		IOConfig.followerIDs = new int[] {31}; //TODO magic numbers!!
		return IOConfig;
	}
}
