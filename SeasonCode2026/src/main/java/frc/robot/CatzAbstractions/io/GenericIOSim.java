package frc.robot.CatzAbstractions.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Utilities.MotorUtil.Gains;
import java.util.function.UnaryOperator;

public class GenericIOSim<T extends GenericMotorIO.MotorIOInputs> implements GenericMotorIO<T> {

    // Defaults to 1 Kraken, but you can adjust the gearbox/motor count here
    private final DCMotor gearBox = DCMotor.getKrakenX60Foc(1);
    // Simulation Plant: Assuming a generic reduction or using constants if available
    private final double GEAR_REDUCTION; // Example reduction
    private final double J_KG_M2 = 0.025; // Moment of Inertia

    private final DCMotorSim motorSim;

    // --- Control Logic ---
    private final PIDController pid;

    // State Tracking
    private double currentVoltage = 0.0;
    private double targetSetpoint = 0.0;
    private ControlMode currentControlMode = ControlMode.VOLTAGE;
    private boolean isBrakeMode = true;

    // Soft Limits (Simulation only)
    private boolean softLimitsEnabled = false;
    private double forwardSoftLimit = Double.MAX_VALUE;
    private double reverseSoftLimit = -Double.MAX_VALUE;

    private TalonFXConfiguration simConfig = new TalonFXConfiguration();

    private enum ControlMode {
        VOLTAGE,
        DUTY_CYCLE,
        POSITION,
        VELOCITY,
        MOTION_MAGIC // Treated as Position PID for basic sim
    }

    public GenericIOSim() {
        this(1, new Gains(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    public GenericIOSim(double gearReduction, Gains gains) {
        this.GEAR_REDUCTION = gearReduction;
        this.pid = new PIDController(gains.kP(), gains.kI(), gains.kD());

        // Using a standard DC motor system
        var plant = LinearSystemId.createDCMotorSystem(gearBox, J_KG_M2, GEAR_REDUCTION);
        motorSim = new DCMotorSim(plant, gearBox, GEAR_REDUCTION - 0.01, GEAR_REDUCTION + 0.01);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // 1. Run Closed-Loop Control Logic (Simulating the TalonFX onboard computer)
        if (currentControlMode == ControlMode.POSITION || currentControlMode == ControlMode.MOTION_MAGIC) {
            // Calculate voltage to get to position
            // Note: motorSim.getAngularPosition() returns Radians
            // We assume setpoints are in Rotations (as per GenericTalonFXIOReal)
            double currentRotations = Units.radiansToRotations(motorSim.getAngularPositionRad());
            double pidOutput = pid.calculate(currentRotations, targetSetpoint);

            // Add kF/Feedforward logic here if desired (e.g., kG for arms)
            currentVoltage = pidOutput;

        } else if (currentControlMode == ControlMode.VELOCITY) {
            double currentRps = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());
            double pidOutput = pid.calculate(currentRps, targetSetpoint);
            currentVoltage = pidOutput + (targetSetpoint * simConfig.Slot0.kV); // Simple kV Feedforward
        }

        // 2. Handle Soft Limits
        if (softLimitsEnabled) {
            double currentPos = Units.radiansToRotations(motorSim.getAngularPositionRad());
            if (currentPos > forwardSoftLimit && currentVoltage > 0) {
                currentVoltage = 0;
            } else if (currentPos < reverseSoftLimit && currentVoltage < 0) {
                currentVoltage = 0;
            }
        }

        // Apply Voltage to Physics Sim
        // Clamp to battery voltage
        currentVoltage = MathUtil.clamp(currentVoltage, -12.0, 12.0);

        motorSim.setInputVoltage(currentVoltage);

        // Step the Simulation
        motorSim.update(0.02); // Standard loop time 20ms

        // Update Inputs object
        inputs.isLeaderConnected = true;
        inputs.isFollowerConnected = new boolean[0]; // No followers simulated here

        // Convert Sim Units (Radians) to Robot Units (Rotations)
        inputs.position = Units.radiansToRotations(motorSim.getAngularPositionRad());
        inputs.velocityRPS = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());
        // Simple derivative for acceleration

        inputs.appliedVolts = new double[] { currentVoltage };
        inputs.supplyCurrentAmps = new double[] { motorSim.getCurrentDrawAmps() };
        inputs.torqueCurrentAmps = new double[] { motorSim.getCurrentDrawAmps() };
        inputs.tempCelcius = new double[] { 45.0 }; // Dummy temp
    }

    @Override
    public void stop() {
        currentControlMode = ControlMode.VOLTAGE;
        currentVoltage = 0.0;
        targetSetpoint = 0.0;
    }

    @Override
    public void setVoltageSetpoint(double voltage) {
        currentControlMode = ControlMode.VOLTAGE;
        this.currentVoltage = voltage;
    }

    @Override
    public void setDutyCycleSetpoint(double percent) {
        currentControlMode = ControlMode.DUTY_CYCLE;
        this.currentVoltage = percent * 12.0; // Assume 12V battery
    }

    @Override
    public void setMotionMagicSetpoint(double mechanismPosition) {
        currentControlMode = ControlMode.MOTION_MAGIC;
        this.targetSetpoint = mechanismPosition;
    }

    @Override
    public void setVelocitySetpoint(double mechanismVelocity) {
        currentControlMode = ControlMode.VELOCITY;
        this.targetSetpoint = mechanismVelocity;
    }

    @Override
    public void setPositionSetpoint(double mechanismPosition) {
        currentControlMode = ControlMode.POSITION;
        this.targetSetpoint = mechanismPosition;
    }

    @Override
    public void setCurrentPosition(double mechanismPosition) {
        // Reset the simulation state to a specific position
        // Sim uses Radians
        double rads = Units.rotationsToRadians(mechanismPosition);
        motorSim.setState(rads, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void useSoftLimits(boolean enable) {
        this.softLimitsEnabled = enable;
        UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return config;
        };
        changeMainConfig(configChanger);
    }

    @Override
    public void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {
        // Update the simulated PID controller
        pid.setPID(p, i, d);

        // Update internal config storage
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
    public void setMotionMagicParameters(double velocity, double acceleration, double jerk) {
        // In a high-fidelity sim, you would configure a TrapezoidProfile here.
        // For basic testing, we often stick to standard PID, but we store the values.
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
        // In sim, we only have one main motor representation
        setNeutralBrake(neutralMode == NeutralModeValue.Brake);
    }

    @Override
    public void setNeutralBrake(boolean wantsBrake) {
        this.isBrakeMode = wantsBrake;
        simConfig.MotorOutput.NeutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }

    public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        simConfig = configChanger.apply(simConfig);

        // Update local limits from config
        this.forwardSoftLimit = simConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
        this.reverseSoftLimit = simConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }
}
