package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeDeployIOSim extends GenericIOSim<IntakeDeployIO.IntakeDeployIOInputs> implements IntakeDeployIO{
    public IntakeDeployIOSim(Gains gains){
        super(gains);
    }
}
