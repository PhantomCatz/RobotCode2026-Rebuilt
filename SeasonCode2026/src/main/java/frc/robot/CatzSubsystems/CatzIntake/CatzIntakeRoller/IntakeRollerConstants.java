package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeRollerConstants {

	public static final Setpoint OFF_SETPOINT = Setpoint.withVoltageSetpoint(0.0);
	// public static final Setpoint ON_SETPOINT = Setpoint.withVoltageSetpoint(6.7);
	public static final Setpoint ON_SETPOINT = Setpoint.withVoltageSetpoint(7.0);
	public static final Setpoint S_SETPOINT = Setpoint.withDutyCycleSetpoint(0.7);
	public static final Setpoint JIGGLE_SETPOINT = Setpoint.withVoltageSetpoint(3.0);

	// public static final LoggedTunableNumber ON_SETPOINT_LOG = new LoggedTunableNumber("IntakeRollers/Voltage", ON_SETPOINT.baseUnits);
	// public static final LoggedTunableNumber JIGGLE_SETPOINT_LOG = new LoggedTunableNumber("IntakeRollers/Jiggle Volts", JIGGLE_SETPOINT.baseUnits);

    public static final Gains gains = switch (CatzConstants.getRobotType()) {
        case SN1 -> new Gains(0.0, 0, 0.0, 0.0, 0.0, 0, 0.0);
        case SN2 -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SN_TEST -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };

	private static final int INTAKE_MOTOR_ID = 31;
	private static final double NO_MOVE_INTAKE_SPEED = 5.0; // TODO make this right
	private static final double INTAKE_SPEED_SLOPE = 0.3; // TODO make this right

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
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;


		FXConfig.Feedback.SensorToMechanismRatio = 0.0; //TODO dont use magic number

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = INTAKE_MOTOR_ID;
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

	public static Setpoint getOnSetpoint() {
		ChassisSpeeds speeds = CatzRobotTracker.Instance.getRobotRelativeChassisSpeeds();
		double vx = speeds.vxMetersPerSecond;
		double vy = speeds.vyMetersPerSecond;
		double driveDirection = Math.atan2(vy, vx);
		double intakeDirection = CatzRobotTracker.Instance.getEstimatedPose().getRotation().getRadians();
		double angleBetween = Math.abs(MathUtil.angleModulus(intakeDirection-driveDirection));
		if (angleBetween > Math.PI/2.0) {
			return Setpoint.withVoltageSetpoint(NO_MOVE_INTAKE_SPEED);
		}
		double robotSpeed = Math.hypot(vx, vy);
		double volts = NO_MOVE_INTAKE_SPEED + robotSpeed*INTAKE_SPEED_SLOPE;
		return Setpoint.withVoltageSetpoint(volts);
	}
}
