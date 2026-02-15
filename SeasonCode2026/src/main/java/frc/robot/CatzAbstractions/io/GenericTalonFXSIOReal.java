package frc.robot.CatzAbstractions.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class GenericTalonFXSIOReal<T extends GenericMotorIO.MotorIOInputs> implements GenericMotorIO<T>  {

    // initialize follower if needed
    private TalonFXS leaderTalon;
    private TalonFXS[] followerTalons;

    private TalonFXSConfiguration config = new TalonFXSConfiguration();
    private TalonFXSConfiguration followerConfig = new TalonFXSConfiguration();

    private final StatusSignal<Angle> internalPositionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final List<StatusSignal<Voltage>> appliedVoltage;
    private final List<StatusSignal<Current>> supplyCurrent;
    private final List<StatusSignal<Current>> torqueCurrent;
    private final List<StatusSignal<Temperature>> tempCelsius;

    private ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    private static double Final_Ratio;

	private CANBus followerTalonCANBus = new CANBus("*");
	private CANBus leaderTalonCANBus = new CANBus("*");

    /**
     * base for constructors
     * 1 motor sets bare minimum to not kill itself
     * User must set MotionMagic, current limits, etc after instantiation
     * @param leader motor
     * @param s0g slot 0 gains
     */
    public GenericTalonFXSIOReal(MotorIOTalonFXSConfig config) {

		requestGetter = config.requestGetter;
		leaderTalon = new TalonFXS(config.mainID, leaderTalonCANBus);
		setMainConfig(config.mainConfig);

		if(config.followerIDs.length != 0) {
			followerTalons = new TalonFXS[config.followerIDs.length];
			for (int i = 0; i < config.followerIDs.length; i++) {
				followerTalons[i] = new TalonFXS(config.followerIDs[i], followerTalonCANBus);
				followerTalons[i].setControl(new Follower(config.mainID, config.followerValue[i]));
			}
			setFollowerConfig(followerConfig);
		}

        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        acceleration = leaderTalon.getAcceleration();

		if (followerTalons == null || followerTalons.length == 0) {
			appliedVoltage = List.of(leaderTalon.getMotorVoltage());
			supplyCurrent = List.of(leaderTalon.getSupplyCurrent());
			torqueCurrent = List.of(leaderTalon.getTorqueCurrent());
			tempCelsius   = List.of(leaderTalon.getDeviceTemp());
		} else {
			var applied = new ArrayList<StatusSignal<Voltage>>();
			var supply  = new ArrayList<StatusSignal<Current>>();
			var torque  = new ArrayList<StatusSignal<Current>>();
			var temps   = new ArrayList<StatusSignal<Temperature>>();

			applied.add(leaderTalon.getMotorVoltage());
			supply.add(leaderTalon.getSupplyCurrent());
			torque.add(leaderTalon.getTorqueCurrent());
			temps.add(leaderTalon.getDeviceTemp());

			for (TalonFXS talon : followerTalons) {
				applied.add(talon.getMotorVoltage());
				supply.add(talon.getSupplyCurrent());
				torque.add(talon.getTorqueCurrent());
				temps.add(talon.getDeviceTemp());
			}

			appliedVoltage = List.copyOf(applied);
			supplyCurrent  = List.copyOf(supply);
			torqueCurrent  = List.copyOf(torque);
			tempCelsius    = List.copyOf(temps);
		}

    }


    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.isLeaderConnected =
            BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                acceleration,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();

		if (followerTalons != null && followerTalons.length > 0) {
			inputs.isFollowerConnected = new boolean[followerTalons.length];
			for (int i = 0; i < followerTalons.length; i++) {
				inputs.isFollowerConnected[i] = BaseStatusSignal.refreshAll(
					appliedVoltage.get(i + 1),
					supplyCurrent.get(i + 1),
					torqueCurrent.get(i + 1),
					tempCelsius.get(i + 1)
				).isOK();
			}
		} else {
			inputs.isFollowerConnected = new boolean[0];
		}

        inputs.position = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
        inputs.velocityRPS = velocityRps.getValueAsDouble() * Final_Ratio;
        inputs.accelerationRPS = acceleration.getValueAsDouble() * Final_Ratio;
        inputs.appliedVolts = appliedVoltage.stream()
                                            .mapToDouble(StatusSignal::getValueAsDouble)
                                            .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.tempCelcius = tempCelsius.stream()
                                        .mapToDouble(StatusSignal::getValueAsDouble)
                                        .toArray();


    }

	/**
	 * Sets the talon to a duty cycle out of 0.
	 */
    @Override
    public void stop() {
        leaderTalon.setControl(new DutyCycleOut(0.0));
    }

		/**
	 * Takes a control request and applies to the talon.
	 * 
	 * @param request Control request tht is applied to talon 
	 */
    private void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

	/**
	 * Changes the currently applied main TalonFXConfiguration and applies the new configuration to the main motor.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXSConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

	/**
	 * Applies a voltage setpoint to the leader and follower talons.
	 *
	 * @param voltage double of voltage to apply
	 */
    @Override
	public void setVoltageSetpoint(double voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	/**
	 * Applies a DutyCycle setpoint to the leader and follower talons.
	 *
	 * @param percent double of percent power to apply 0 to 1
	 */
	@Override
	public void setDutyCycleSetpoint(double percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	/**
	 * Applies a Motion Magic setpoint to the leader and follower talons.
	 *
	 * @param mechanismPosition double of position to apply
	 */
	@Override
	public void setMotionMagicSetpoint(double mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	/**
	 * Applies a voltage setpoint to the leader and follower talons.
	 *
	 * @param percent double of velocity to apply
	 */
	@Override
	public void setVelocitySetpoint(double mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	/**
	 * Applies a position setpoint to the leader and follower talons.
	 *
	 * @param mechanismPosition double of position to apply
	 */
	@Override
	public void setPositionSetpoint(double mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	/**
	 * Sets the motor encoder to the desired value.
	 *
	 * @param mechanismPosition double of position to apply to the encoder
	 */
	@Override
	public void setCurrentPosition(double mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}

	/**
	 * Applies a configuration to a specified talon.
	 * 
	 * @param fx talon to have its configuration changed
	 * @param config configuration to be applied
	 */
	public void applyConfig(TalonFXS fx, TalonFXSConfiguration config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fx.getConfigurator().apply(config);
				if (result.isOK()) {
					break;
				}
			}
		});
	}

	/**
	 * Changes whether soft limits are being used or not.
	 * 
	 * @param enable whether to use soft limits or not
	 */
    @Override
	public void useSoftLimits(boolean enable) {
		UnaryOperator<TalonFXSConfiguration> configChanger = (config) -> {
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
			return config;
		};

		changeMainConfig(configChanger);
	}

	/**
	 * Applies a TalonFXConfiguration to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(TalonFXSConfiguration configuration) {
		config = configuration;
		applyConfig(leaderTalon, config);
	}

	/**
	 * Applies a TalonFXConfiguration to all follower motors.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setFollowerConfig(TalonFXSConfiguration configuration) {
		followerConfig = configuration;
		for (TalonFXS talon : followerTalons) {
			applyConfig(talon, followerConfig);
		}
	}

	/**
     * Configures the gains for Slot 0, used for closed-loop control.
     *
     * @param p proportional gain kP. Corrects error by a factor of the current offset
     * @param i integral gain kI. Corrects accumulated error over time
     * @param d derivative gain kD. Corrects based on the rate of change of the error
     * @param s static friction feedforward kS. Voltage to overcome friction
     * @param v velocity feedforward kV. Voltage to maintain a target velocity
     * @param a acceleration feedforward kA. Voltage to achieve target acceleration
     * @param g gravity feedforward kG. Voltage to counteract gravity based on position
     */
	@Override
	public void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {
		UnaryOperator<TalonFXSConfiguration> configChanger = (config) -> {
			config.Slot0.kP = p;
			config.Slot0.kI = i;
			config.Slot0.kD = d;
			config.Slot0.kS = s;
			config.Slot0.kV = v;
			config.Slot0.kA = a;
			config.Slot0.kG = g;
			return config;
		};

		changeMainConfig(configChanger);
	}

	/**
     * Configures the gains for Slot 1, used for closed-loop control.
     *
     * @param p proportional gain kP. Corrects error by a factor of the current offset
     * @param i integral gain kI. Corrects accumulated error over time
     * @param d derivative gain kD. Corrects based on the rate of change of the error
     * @param s static friction feedforward kS. Voltage to overcome friction
     * @param v velocity feedforward kV. Voltage to maintain a target velocity
     * @param a acceleration feedforward kA. Voltage to achieve target acceleration
     * @param g gravity feedforward kG. Voltage to counteract gravity based on position
     */
	@Override
	public void setGainsSlot1(double p, double i, double d, double s, double v, double a, double g) {
		UnaryOperator<TalonFXSConfiguration> configChanger = (config) -> {
			config.Slot1.kP = p;
			config.Slot1.kI = i;
			config.Slot1.kD = d;
			config.Slot1.kS = s;
			config.Slot1.kV = v;
			config.Slot1.kA = a;
			config.Slot1.kG = g;
			return config;
		};

		changeMainConfig(configChanger);
	}

	@Override
	public void setMotionMagicParameters(double velocity, double acceleration, double jerk) {
		UnaryOperator<TalonFXSConfiguration> configChanger = (config) -> {
			config.MotionMagic.MotionMagicCruiseVelocity = velocity;
			config.MotionMagic.MotionMagicAcceleration = acceleration;
			config.MotionMagic.MotionMagicJerk = jerk;
			return config;
		};

		changeMainConfig(configChanger);
	}

	/**
	 * Sets the nuetral mode for a specified talon.
	 * 
	 * @param fx talon to have its nuertal mode changed
	 * @param config nuetral mode to be applied
	 */
    @Override
	public void setNeutralMode(TalonFXS fx, NeutralModeValue neutralMode) {
		threadPoolExecutor.submit(() -> {
			fx.setNeutralMode(neutralMode);
		});
	}

	/**
	 * Configuration for a MotorIOTalonFXS. Motion magic control is on slot 0, velocity on slot 1, and position PID on slot 2.
	 */
	public static class MotorIOTalonFXSConfig {
		public int mainID = -1;
		public String mainBus = "ASSIGN_BUS";
		public TalonFXSConfiguration mainConfig = new TalonFXSConfiguration();
		public int[] followerIDs = new int[0];
		public String[] followerBuses = new String[0];
		public TalonFXSConfiguration followerConfig = new TalonFXSConfiguration();
		public MotorAlignmentValue[] followerValue = new MotorAlignmentValue[0];
		public ControlRequestGetter requestGetter = new ControlRequestGetter();
	}


    public static class ControlRequestGetter {
		public ControlRequest getVoltageRequest(double voltage) {
			return new VoltageOut(voltage);
		}

		public ControlRequest getDutyCycleRequest(double percent) {
			return new DutyCycleOut(percent);
		}

		public ControlRequest getMotionMagicRequest(double mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(double mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(double mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}

}
