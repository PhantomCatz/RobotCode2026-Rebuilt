package frc.robot.CatzAbstractions.io;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalInOutIO {

    @AutoLog
    public static class DigitalInOutIOInputs {
        public boolean value = false;    // The current state of the DIO (High/Low)
        public boolean isOutput = false; // Logs if the pin is currently configured as output
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DigitalInOutIOInputs inputs) {}

    /**
     * Sets the digital output value.
     * NOTE: This is only valid if the IO is configured as an output.
     * * @param value true for High (5V/3.3V), false for Low (0V)
     */
    public default void setOutput(boolean value) {}
}
