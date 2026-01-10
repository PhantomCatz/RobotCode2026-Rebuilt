package frc.robot.CatzAbstractions.io;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class DigitalInOutIOSim implements DigitalInOutIO {

    private final DIOSim dioSim;
    private boolean isOutputMode;
    private boolean currentSimValue = false;

    public DigitalInOutIOSim(int channel, boolean isOutputMode) {
        this.isOutputMode = isOutputMode;
        // DIOSim works for both inputs and outputs
        this.dioSim = new DIOSim(channel);
        this.dioSim.setIsInput(!isOutputMode);
    }

    @Override
    public void updateInputs(DigitalInOutIOInputs inputs) {
        inputs.isOutput = isOutputMode;

        // In sim, we read the value from the simulator data
        inputs.value = dioSim.getValue();
    }

    @Override
    public void setOutput(boolean value) {
        if (isOutputMode) {
            this.currentSimValue = value;
            dioSim.setValue(value);
        }
    }

    /**
     * Helper method for Sim-only interactions (e.g., unit tests or physics sim)
     * to artificially trigger the sensor.
     */
    public void setSimSensorTriggered(boolean triggered) {
        if (!isOutputMode) {
            dioSim.setValue(triggered);
        }
    }
}
