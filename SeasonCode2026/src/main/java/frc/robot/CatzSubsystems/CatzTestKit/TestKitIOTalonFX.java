package frc.robot.CatzSubsystems.CatzTestKit;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class TestKitIOTalonFX extends GenericTalonFXIOReal<TestKitIO.TestKitIOInputs> implements TestKitIO{
    public TestKitIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

}
