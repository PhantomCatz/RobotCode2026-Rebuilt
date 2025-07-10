package frc.robot.CatzSubsystems.CatzArm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double positionDegreesFinalShaft = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void runSetpointUp(double setpointDegrees) {}

    public default void runSetpointDown(double setpointDegrees) {}
}
