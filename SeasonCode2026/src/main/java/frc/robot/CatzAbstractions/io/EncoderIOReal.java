package frc.robot.CatzAbstractions.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class EncoderIOReal implements EncoderIO {

    private final CANcoder cancoder;

    // Signals for efficient data retrieval
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;

    /**
     * @param deviceId CAN ID of the CANcoder
     * @param canbus Name of the CAN bus (e.g., "rio", "canivore")
     * @param magnetOffsetRotations Offset to zero the mechanism (0.0 to 1.0)
     */
    public EncoderIOReal(int deviceId, String canbus, double magnetOffsetRotations) {
        cancoder = new CANcoder(deviceId, canbus);

        // Configure the CANcoder
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = magnetOffsetRotations;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        cancoder.getConfigurator().apply(config);

        // Initialize signals
        positionSignal = cancoder.getAbsolutePosition();
        velocitySignal = cancoder.getVelocity();
        voltageSignal = cancoder.getSupplyVoltage();

        // Optimize bus traffic (sets update frequency)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, positionSignal, velocitySignal, voltageSignal);
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        // Refresh all signals synchronously
        var status = BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            voltageSignal
        );

        inputs.isConnected = status.isOK();

        inputs.absolutePositionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRPS = velocitySignal.getValueAsDouble();
        inputs.supplyVoltage = voltageSignal.getValueAsDouble();

        // Retrieve faults
        inputs.faultHardware = cancoder.getFault_Hardware().getValue();
        inputs.faultUndervoltage = cancoder.getFault_Undervoltage().getValue();
    }

    @Override
    public void setPosition(double positionRotations) {
        cancoder.setPosition(positionRotations);
    }
}
