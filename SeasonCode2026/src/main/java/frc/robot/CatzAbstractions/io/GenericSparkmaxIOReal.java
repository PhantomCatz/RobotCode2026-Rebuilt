package frc.robot.CatzAbstractions.io;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

// 2025 REVLib Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;

public abstract class GenericSparkmaxIOReal implements GenericMotorIO {

    private final SparkMax leaderMotor;
    private final SparkMax[] followerMotors;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    private final double gearRatio;

    // We maintain a local copy of the config to modify and re-apply as needed
    private final SparkMaxConfig sparkConfig = new SparkMaxConfig();

    // Thread pool for configuration actions to avoid blocking the main loop
    private final BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private final ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, TimeUnit.MILLISECONDS, queue);

    public GenericSparkmaxIOReal(MotorIOSparkMaxConfig config) {
        this.gearRatio = config.gearRatio;

        // 1. Initialize Leader
        leaderMotor = new SparkMax(config.mainID, MotorType.kBrushless);
        encoder = leaderMotor.getEncoder();
        closedLoopController = leaderMotor.getClosedLoopController();

        // 2. Build Initial Configuration
        sparkConfig.inverted(config.invertMotor)
                   .smartCurrentLimit(config.currentLimitAmps)
                   .voltageCompensation(12.0);

        // Configure Feedback Sensor (Encoder)
        sparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        sparkConfig.signals.primaryEncoderVelocityAlwaysOn(true); // Optimization for reading velocity

        // 3. Initialize Followers
        if (config.followerIDs.length > 0) {
            followerMotors = new SparkMax[config.followerIDs.length];
            for (int i = 0; i < config.followerIDs.length; i++) {
                followerMotors[i] = new SparkMax(config.followerIDs[i], MotorType.kBrushless);

                // Create a follower config
                SparkMaxConfig followerConfig = new SparkMaxConfig();
                followerConfig.follow(leaderMotor, config.followerOpposeMain[i])
                              .smartCurrentLimit(config.currentLimitAmps);

                // Apply config to follower
                followerMotors[i].configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        } else {
            followerMotors = new SparkMax[0];
        }

        // 4. Apply Initial Config to Leader
        // We use ResetSafeParameters to clear old state, and NoPersist to avoid wearing out flash during dev
        leaderMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // Connectivity check (simulated)
        inputs.isLeaderConnected = true;

        // Position: REV rotations -> Adjusted rotations
        inputs.position = encoder.getPosition() * gearRatio;

        // Velocity: REV RPM -> RPS -> Adjusted RPS
        inputs.velocityRPS = (encoder.getVelocity() / 60.0) * gearRatio;

        // Acceleration: SparkMax does not provide raw acceleration signal
        inputs.accelerationRPS = 0.0;

        // Electrical
        double busVoltage = leaderMotor.getBusVoltage();
        double appliedOutput = leaderMotor.getAppliedOutput();

        inputs.appliedVolts = new double[] { appliedOutput * busVoltage };
        inputs.supplyCurrentAmps = new double[] { leaderMotor.getOutputCurrent() };
        inputs.torqueCurrentAmps = new double[] { leaderMotor.getOutputCurrent() };
        inputs.tempCelcius = new double[] { leaderMotor.getMotorTemperature() };

        // Handle Followers
        inputs.isFollowerConnected = new boolean[followerMotors.length];
        for (int i = 0; i < followerMotors.length; i++) {
            inputs.isFollowerConnected[i] = true;
        }
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
    }

    @Override
    public void setVoltageSetpoint(double voltage) {
        leaderMotor.setVoltage(voltage);
    }

    @Override
    public void setDutyCycleSetpoint(double percent) {
        System.out.println("hiii");
        leaderMotor.set(percent);
    }

    @Override
    public void setMotionMagicSetpoint(double mechanismPosition) {
        // Map MotionMagic to REV SmartMotion (Slot 0)
        double targetRotations = mechanismPosition / gearRatio;
        closedLoopController.setReference(targetRotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setVelocitySetpoint(double mechanismVelocity) {
        // REV units are RPM (Slot 1)
        double targetRPM = (mechanismVelocity / gearRatio) * 60.0;
        closedLoopController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setPositionSetpoint(double mechanismPosition) {
        // REV units are Rotations (Slot 0)
        double targetRotations = mechanismPosition / gearRatio;
        closedLoopController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setCurrentPosition(double mechanismPosition) {
        // Execute on thread to avoid blocking loop
        threadPoolExecutor.submit(() -> {
            encoder.setPosition(mechanismPosition / gearRatio);
        });
    }

    @Override
    public void useSoftLimits(boolean enable) {
        // Modify local config and re-apply
        applyConfigChange(config -> {
            config.softLimit.forwardSoftLimitEnabled(enable);
            config.softLimit.reverseSoftLimitEnabled(enable);
        });
    }

    /**
     * Helper to apply config changes to the SparkMax.
     * Uses NoPersistParameters to allow for fast runtime tuning without flash wear.
     */
    private void applyConfigChange(Consumer<SparkMaxConfig> configOps) {
        threadPoolExecutor.submit(() -> {
            // Apply changes to the local config object
            configOps.accept(sparkConfig);
            // Push to motor (NoReset, NoPersist)
            leaderMotor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    @Override
    public void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {
        applyConfigChange(config -> {
            config.closedLoop.p(p, ClosedLoopSlot.kSlot0)
                  .i(i, ClosedLoopSlot.kSlot0)
                  .d(d, ClosedLoopSlot.kSlot0)
                  .velocityFF(v, ClosedLoopSlot.kSlot0) // Mapping kV to FF
                  .iZone(0.0, ClosedLoopSlot.kSlot0);
        });
    }

    @Override
    public void setGainsSlot1(double p, double i, double d, double s, double v, double a, double g) {
        applyConfigChange(config -> {
            config.closedLoop.p(p, ClosedLoopSlot.kSlot1)
                  .i(i, ClosedLoopSlot.kSlot1)
                  .d(d, ClosedLoopSlot.kSlot1)
                  .velocityFF(v, ClosedLoopSlot.kSlot1)
                  .iZone(0.0, ClosedLoopSlot.kSlot1);
        });
    }

    @Override
    public void setMotionMagicParameters(double velocity, double acceleration, double jerk) {
        // Convert RPS to RPM
        double cruiseVelRPM = (velocity / gearRatio) * 60.0;
        // Convert RPS^2 to RPM/s
        double maxAccRPMs = (acceleration / gearRatio) * 60.0;

        applyConfigChange(config -> {
            // Smart Motion is configured under closedLoop.smartMotion
            config.closedLoop.maxMotion.maxVelocity(cruiseVelRPM, ClosedLoopSlot.kSlot0)
                  .maxAcceleration(maxAccRPMs, ClosedLoopSlot.kSlot0)
                  .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0);
        });
    }


    @Override
    public void setNeutralBrake(boolean wantsBrake) {
        IdleMode mode = wantsBrake ? IdleMode.kBrake : IdleMode.kCoast;

        threadPoolExecutor.submit(() -> {
            // Update Leader Config
            sparkConfig.idleMode(mode);
            leaderMotor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            // Update Followers
            for (SparkMax follower : followerMotors) {
                SparkMaxConfig followerConfig = new SparkMaxConfig();
                // We must re-apply "follow" when configuring, or use the accessor if available.
                // Safest to just configure IdleMode on a fresh config for followers to avoid overwriting "follow" if we aren't careful,
                // BUT in 2025, `configure` overwrites. We need to ensure follow is maintained.
                // Assuming followers were set up once in constructor, we just push IdleMode.
                followerConfig.idleMode(mode)
                              .follow(leaderMotor); // Re-assert follow
                follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        });
    }

    /**
     * Configuration class for SparkMax
     */
    public static class MotorIOSparkMaxConfig {
        public int mainID = -1;
        public int[] followerIDs = new int[0];
        public boolean[] followerOpposeMain = new boolean[0];

        public boolean invertMotor = false;
        public int currentLimitAmps = 40;
        public double gearRatio = 1.0;
    }
}
