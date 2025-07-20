package frc.robot.CatzSubsystems.CatzArm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double positionDegreesFinalShaft = 0.0;
        public boolean isArmMotorConnected = false;
        public boolean isBotLimitSwitched = false;
        public double velocityRPM = 0.0;
        public double absoluteEncoderPositionRads = 0.0;
        public double relativeEncoderPositionRads = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void runSetpointUp(double setpointDegrees, double feedforwardVolts) {}

    public default void runSetpointDown(double setpointDegrees, double feedforwardVolts) {}

    public default void setPercentOutput(double percentOutput) {}

    public default void resetPosition(double pos) {}

    public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public default void runCharacterizationMotor(double input) {}
}
