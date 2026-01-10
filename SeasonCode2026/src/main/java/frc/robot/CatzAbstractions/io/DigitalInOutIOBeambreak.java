package frc.robot.CatzAbstractions.io;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalInOutIOBeambreak implements DigitalInOutIO {

    private DigitalInput input;
    private DigitalOutput output;
    private final int channel;
    private boolean isOutputMode;
    private boolean currentOutputValue = false;

    /**
     * Constructor for DigitalInOutIOReal.
     * * @param channel The DIO port number on the RoboRIO.
     * @param outputModeEnabled Set to true if this pin controls an output (LED, etc.),
     * false if it reads a sensor (Limit Switch, Beam Break).
     */
    public DigitalInOutIOBeambreak(int channel, boolean outputModeEnabled) {
        this.channel = channel;
        this.isOutputMode = outputModeEnabled;

        if (outputModeEnabled) {
            output = new DigitalOutput(channel);
        } else {
            input = new DigitalInput(channel);
        }
    }

    @Override
    public void updateInputs(DigitalInOutIOInputs inputs) {
        inputs.isOutput = isOutputMode;

        if (isOutputMode) {
            // If we are an output, the "input" value is just whatever we last wrote
            inputs.value = currentOutputValue;
        } else {
            // You may need to invert this depending on your wiring, but usually we log raw here.
            inputs.value = input.get();
        }
    }

    @Override
    public void setOutput(boolean value) {
        if (isOutputMode) {
            output.set(value);
            currentOutputValue = value;
        } else {
            System.err.println("Warning: Attempted to set output on Input-configured DIO channel " + channel);
        }
    }
}
