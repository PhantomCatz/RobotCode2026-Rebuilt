package frc.robot.CatzSubsystems.CatzIntakeDeploy;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeDeployIOTalonFX extends GenericTalonFXIOReal<IntakeDeployIO.IntakeDeployIOInputs> implements IntakeDeployIO{
    public IntakeDeployIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

}
