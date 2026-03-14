package frc.robot.CatzSubsystems.CatzTestKit;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class TestKitIOSim extends GenericIOSim<TestKitIO.TestKitIOInputs> implements TestKitIO{
    public TestKitIOSim(Gains gains){
        super(gains);
    }
}
