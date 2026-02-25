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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.UnaryOperator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class GenericTalonFXIOReal<T extends GenericMotorIO.MotorIOInputs> implements GenericMotorIO<T> {

	// initialize follower if needed
	protected TalonFX leaderTalon;
	protected TalonFX[] followerTalons;

	private TalonFXConfiguration config = new TalonFXConfiguration();
	private TalonFXConfiguration followerConfig = new TalonFXConfiguration();

	protected final BaseStatusSignal[] allSignals;

	protected final StatusSignal<Angle> internalPositionRotations;
	protected final StatusSignal<AngularVelocity> velocityRps;
	protected final StatusSignal<AngularAcceleration> acceleration;
	protected final List<StatusSignal<Voltage>> appliedVoltage;
	protected final List<StatusSignal<Current>> supplyCurrent;
	protected final List<StatusSignal<Current>> torqueCurrent;
	protected final List<StatusSignal<Temperature>> tempCelsius;

	// OPTIMIZATION: Share one thread pool for all motors to reduce resource usage
	private static final ExecutorService configExecutor = Executors.newFixedThreadPool(1);

	// OPTIMIZATION: Cache ControlRequests to avoid creating "new" objects every
	// loop
	protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
	protected final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
	protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
	protected final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
	protected final VelocityVoltage velocityVoltRequest = new VelocityVoltage(0).withSlot(0);
	protected final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0).withSlot(0);

	private final boolean[] connectedBuffer;

	public GenericTalonFXIOReal(MotorIOTalonFXConfig config) {
		leaderTalon = new TalonFX(config.mainID, new CANBus(config.mainBus));
		setMainConfig(config.mainConfig);

		if (config.followerIDs.length != 0) {
			followerTalons = new TalonFX[config.followerIDs.length];
			for (int i = 0; i < config.followerIDs.length; i++) {
				followerTalons[i] = new TalonFX(config.followerIDs[i], new CANBus(config.mainBus));
				followerTalons[i].setControl(new Follower(config.mainID, config.followerAlignmentValue[i]));
			}
			setFollowerConfig(followerConfig);
		}

		internalPositionRotations = leaderTalon.getPosition();
		velocityRps = leaderTalon.getVelocity();
		acceleration = leaderTalon.getAcceleration();

		var applied = new ArrayList<StatusSignal<Voltage>>();
		var supply = new ArrayList<StatusSignal<Current>>();
		var torque = new ArrayList<StatusSignal<Current>>();
		var temps = new ArrayList<StatusSignal<Temperature>>();

		applied.add(leaderTalon.getMotorVoltage());
		supply.add(leaderTalon.getSupplyCurrent());
		torque.add(leaderTalon.getTorqueCurrent());
		temps.add(leaderTalon.getDeviceTemp());

		if (followerTalons != null) {
			for (TalonFX talon : followerTalons) {
				applied.add(talon.getMotorVoltage());
				supply.add(talon.getSupplyCurrent());
				torque.add(talon.getTorqueCurrent());
				temps.add(talon.getDeviceTemp());
			}
		}

		appliedVoltage = List.copyOf(applied);
		supplyCurrent = List.copyOf(supply);
		torqueCurrent = List.copyOf(torque);
		tempCelsius = List.copyOf(temps);

		List<BaseStatusSignal> signalList = new ArrayList<>();
		signalList.add(internalPositionRotations);
		signalList.add(velocityRps);
		signalList.add(acceleration);
		signalList.addAll(appliedVoltage);
		signalList.addAll(supplyCurrent);
		signalList.addAll(torqueCurrent);
		signalList.addAll(tempCelsius);

		allSignals = signalList.toArray(new BaseStatusSignal[0]);

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, allSignals);
		connectedBuffer = (followerTalons != null) ? new boolean[followerTalons.length] : new boolean[0];
	}

	public GenericTalonFXIOReal(MotorIOTalonFXConfig config, boolean requiresFastUpdate) {
		leaderTalon = new TalonFX(config.mainID, new CANBus(config.mainBus));
		setMainConfig(config.mainConfig);

		if (config.followerIDs.length != 0) {
			followerTalons = new TalonFX[config.followerIDs.length];
			for (int i = 0; i < config.followerIDs.length; i++) {
				followerTalons[i] = new TalonFX(config.followerIDs[i], new CANBus(config.mainBus));
				followerTalons[i].setControl(new Follower(config.mainID, config.followerAlignmentValue[i]));
			}
			setFollowerConfig(followerConfig);
		}

		internalPositionRotations = leaderTalon.getPosition();
		velocityRps = leaderTalon.getVelocity();
		acceleration = leaderTalon.getAcceleration();

		var applied = new ArrayList<StatusSignal<Voltage>>();
		var supply = new ArrayList<StatusSignal<Current>>();
		var torque = new ArrayList<StatusSignal<Current>>();
		var temps = new ArrayList<StatusSignal<Temperature>>();

		applied.add(leaderTalon.getMotorVoltage());
		supply.add(leaderTalon.getSupplyCurrent());
		torque.add(leaderTalon.getTorqueCurrent());
		temps.add(leaderTalon.getDeviceTemp());

		if (followerTalons != null) {
			for (TalonFX talon : followerTalons) {
				applied.add(talon.getMotorVoltage());
				supply.add(talon.getSupplyCurrent());
				torque.add(talon.getTorqueCurrent());
				temps.add(talon.getDeviceTemp());
			}
		}

		appliedVoltage = List.copyOf(applied);
		supplyCurrent = List.copyOf(supply);
		torqueCurrent = List.copyOf(torque);
		tempCelsius = List.copyOf(temps);

		List<BaseStatusSignal> signalList = new ArrayList<>();
		signalList.add(internalPositionRotations);
		signalList.add(velocityRps);
		signalList.add(acceleration);
		signalList.addAll(appliedVoltage);
		signalList.addAll(supplyCurrent);
		signalList.addAll(torqueCurrent);
		signalList.addAll(tempCelsius);

		allSignals = signalList.toArray(new BaseStatusSignal[0]);

		if(requiresFastUpdate){
			BaseStatusSignal.setUpdateFrequencyForAll(100.0, internalPositionRotations, velocityRps);
		}else{
			BaseStatusSignal.setUpdateFrequencyForAll(50.0, allSignals);
		}
		connectedBuffer = (followerTalons != null) ? new boolean[followerTalons.length] : new boolean[0];

	}

	@Override
	public BaseStatusSignal[] getSignals() {
		return allSignals;
	}

	@Override
	public void updateInputs(T inputs) {

		inputs.isLeaderConnected = internalPositionRotations.getStatus().isOK();

		if(followerTalons != null && followerTalons.length > 0) {
			for(int i = 0; i < followerTalons.length; i++) {
				connectedBuffer[i] = appliedVoltage.get(i+1).getStatus().isOK();
			}
		}
		inputs.isFollowerConnected = connectedBuffer;

		inputs.position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(internalPositionRotations, velocityRps);
		inputs.velocityRPS = velocityRps.getValueAsDouble();
		inputs.accelerationRPS = acceleration.getValueAsDouble();

		int count = appliedVoltage.size();

		if (inputs.appliedVolts == null || inputs.appliedVolts.length != count) {
			inputs.appliedVolts = new double[count];
			inputs.supplyCurrentAmps = new double[count];
			inputs.torqueCurrentAmps = new double[count];
			inputs.tempCelcius = new double[count];
		}

		for (int i = 0; i < count; i++) {
			inputs.appliedVolts[i] = appliedVoltage.get(i).getValueAsDouble();
			inputs.supplyCurrentAmps[i] = supplyCurrent.get(i).getValueAsDouble();
			inputs.torqueCurrentAmps[i] = torqueCurrent.get(i).getValueAsDouble();
			inputs.tempCelcius[i] = tempCelsius.get(i).getValueAsDouble();
		}
	}

	@Override
	public void stop() {
		leaderTalon.setControl(dutyCycleRequest.withOutput(0.0));
	}

	@Override
  	public void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
			config.Slot0.kP = p;
			config.Slot0.kI = i;
			config.Slot0.kD = d;
			config.Slot0.kV = v;
			config.Slot0.kA = a;
			config.Slot0.kG = g;
			return config;
		};

		changeAllConfig(configChanger);
	}

	protected void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

	public void changeAllConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		setAllConfig(configChanger.apply(config));
	}

	public void setAllConfig(TalonFXConfiguration configuration) {

		setMainConfig(configuration);

		if (followerTalons != null && followerTalons.length != 0) {

			setFollowerConfig(configuration);

		}

	}

	/**
	 *
	 * Applies a TalonFXConfiguration to all follower motors.
	 *
	 *
	 *
	 * @param configuration Configuration to apply.
	 *
	 */

	public void setFollowerConfig(TalonFXConfiguration configuration) {

		followerConfig = configuration;

		for (TalonFX talon : followerTalons) {

			applyConfig(talon, followerConfig);

		}

	}

	/**
	 *
	 * Applies a TalonFXConfiguration to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 *
	 */

	public void setMainConfig(TalonFXConfiguration configuration) {

		config = configuration;

		applyConfig(leaderTalon, config);

	}

	// ----------------------------------------------------------------------------------
	// OPTIMIZATION: Updated Setters to use Cached Control Requests
	// ----------------------------------------------------------------------------------

	@Override
	public void setVoltageSetpoint(double voltage) {
		leaderTalon.setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setDutyCycleSetpoint(double percent) {
		leaderTalon.setControl(dutyCycleRequest.withOutput(percent));
	}

	@Override
	public void setMotionMagicSetpoint(double mechanismPosition) {
		leaderTalon.setControl(motionMagicRequest.withPosition(mechanismPosition));
	}

	@Override
	public void setVelocitySetpoint(double mechanismVelocity) {
		leaderTalon.setControl(velocityRequest.withVelocity(mechanismVelocity));
	}

	@Override
	public void setVelocitySetpointVoltage(double mechanismVelocity) {
		leaderTalon.setControl(velocityVoltRequest.withVelocity(mechanismVelocity));
	}

	@Override
	public void setPositionSetpoint(double mechanismPosition) {
		leaderTalon.setControl(positionRequest.withPosition(mechanismPosition));
	}

	// ----------------------------------------------------------------------------------
	// Threading and Config
	// ----------------------------------------------------------------------------------

	@Override
	public void setCurrentPosition(double mechanismPosition) {
		configExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}

	public void applyConfig(TalonFX fx, TalonFXConfiguration config) {
		configExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fx.getConfigurator().apply(config);
				if (result.isOK())
					break;
			}
		});
	}

	// ... (Use configExecutor for other config methods too) ...

	@Override
	public void setNeutralMode(TalonFX fx, NeutralModeValue neutralMode) {
		configExecutor.submit(() -> {
			fx.setNeutralMode(neutralMode);
		});
	}

	public static class MotorIOTalonFXConfig {
		public int mainID = -1;
		public String mainBus = "ASSIGN_BUS";
		public TalonFXConfiguration mainConfig = new TalonFXConfiguration();
		public int[] followerIDs = new int[0];
		public String[] followerBuses = new String[0];
		public TalonFXConfiguration followerConfig = new TalonFXConfiguration();
		public MotorAlignmentValue[] followerAlignmentValue = new MotorAlignmentValue[0];
	}
}
