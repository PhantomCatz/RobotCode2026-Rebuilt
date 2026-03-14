package frc.robot.CatzSubsystems.CatzTestKit;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface TestKitIO extends GenericMotorIO<TestKitIO.TestKitIOInputs>{

    @AutoLog
    public static class TestKitIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
