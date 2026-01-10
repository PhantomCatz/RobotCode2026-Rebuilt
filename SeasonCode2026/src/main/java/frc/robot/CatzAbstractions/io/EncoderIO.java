package frc.robot.CatzAbstractions.io;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {

    @AutoLog
    public static class EncoderIOInputs {
        public double absolutePositionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double supplyVoltage = 0.0;

        public boolean isConnected = false;
        public boolean faultHardware = false;
        public boolean faultUndervoltage = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(EncoderIOInputs inputs) {}

    /** Sets the current position (if supported by hardware). */
    public default void setPosition(double positionRotations) {}
}
