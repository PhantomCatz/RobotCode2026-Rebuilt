package frc.robot.CatzAbstractions.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class GenericTalonFXIOReal<T extends GenericMotorIO.MotorIOInputs> implements GenericMotorIO<T> {

    // initialize follower if needed
    protected TalonFX leaderTalon;
    protected TalonFX[] followerTalons;

    private TalonFXConfiguration config = new TalonFXConfiguration();
    private TalonFXConfiguration followerConfig = new TalonFXConfiguration();

    protected final StatusSignal<Angle> internalPositionRotations;
    protected final StatusSignal<AngularVelocity> velocityRps;
    protected final StatusSignal<AngularAcceleration> acceleration;
    protected final List<StatusSignal<Voltage>> appliedVoltage;
    protected final List<StatusSignal<Current>> supplyCurrent;
    protected final List<StatusSignal<Current>> torqueCurrent;
    protected final List<StatusSignal<Temperature>> tempCelsius;

    private ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    /**
     * base for constructors
     * 1 motor sets bare minimum to not kill itself
     * User must set MotionMagic, current limits, etc after instantiation
     * @param leader motor
     * @param s0g slot 0 gains
     */
    public GenericTalonFXIOReal(MotorIOTalonFXConfig config) {

		requestGetter = config.requestGetter;
		leaderTalon = new TalonFX(config.mainID, new CANBus("*"));
		setMainConfig(config.mainConfig);

		if(config.followerIDs.length != 0) {
			followerTalons = new TalonFX[config.followerIDs.length];
			for (int i = 0; i < config.followerIDs.length; i++) {
				followerTalons[i] = new TalonFX(config.followerIDs[i], new CANBus("*"));
				followerTalons[i].setControl(new Follower(config.mainID, config.followerAlignmentValue[i]));
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

			for (TalonFX talon : followerTalons) {
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
    public void updateInputs(T inputs) {
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


        inputs.position = internalPositionRotations.getValueAsDouble(); //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
        inputs.velocityRPS = velocityRps.getValueAsDouble();
        inputs.accelerationRPS = acceleration.getValueAsDouble();
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

    @Override
    public void stop() {
        leaderTalon.setControl(new DutyCycleOut(0.0));
    }

    protected void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

	/**
	 * Changes the currently applied main TalonFXConfiguration and applies the new configuration to the main motor.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

    @Override
	public void setVoltageSetpoint(double voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	public void setDutyCycleSetpoint(double percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	public void setMotionMagicSetpoint(double mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	public void setVelocitySetpoint(double mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	public void setPositionSetpoint(double mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	@Override
	public void setCurrentPosition(double mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}

	public void applyConfig(TalonFX fx, TalonFXConfiguration config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fx.getConfigurator().apply(config);
				if (result.isOK()) {
					break;
				}
			}
		});
	}


    @Override
	public void useSoftLimits(boolean enable) {
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
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
	public void setMainConfig(TalonFXConfiguration configuration) {
		config = configuration;
		applyConfig(leaderTalon, config);
	}

	/**
	 * Applies a TalonFXConfiguration to all follower motors.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setFollowerConfig(TalonFXConfiguration configuration) {
		followerConfig = configuration;
		for (TalonFX talon : followerTalons) {
			applyConfig(talon, followerConfig);
		}
	}

	@Override
	public void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
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

	@Override
	public void setGainsSlot1(double p, double i, double d, double s, double v, double a, double g) {
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
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
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
			config.MotionMagic.MotionMagicCruiseVelocity = velocity;
			config.MotionMagic.MotionMagicAcceleration = acceleration;
			config.MotionMagic.MotionMagicJerk = jerk;
			return config;
		};

		changeMainConfig(configChanger);
	}

    @Override
	public void setNeutralMode(TalonFX fx, NeutralModeValue neutralMode) {
		threadPoolExecutor.submit(() -> {
			fx.setNeutralMode(neutralMode);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		NeutralModeValue neutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.NeutralMode = neutralMode;
		setNeutralMode(leaderTalon, neutralMode);
		for (TalonFX talon : followerTalons) {
			setNeutralMode(talon, neutralMode);
		}
	}




	/**
	 * Configuration for a MotorIOTalonFX. Motion magic control is on slot 0, velocity on slot 1, and position PID on slot 2.
	 */
	public static class MotorIOTalonFXConfig {
		public int mainID = -1;
		public String mainBus = "ASSIGN_BUS";
		public TalonFXConfiguration mainConfig = new TalonFXConfiguration();
		public int[] followerIDs = new int[0];
		public String[] followerBuses = new String[0];
		public TalonFXConfiguration followerConfig = new TalonFXConfiguration();
		public MotorAlignmentValue[] followerAlignmentValue = new MotorAlignmentValue[0];
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
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0);//.withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(double mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(double mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}

		
		public ControlRequest getVoltageRequest(Voltage voltage) {
			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
		}

		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
			return new DutyCycleOut(percent.in(Units.Percent));
		}

		public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(Angle mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}

}
