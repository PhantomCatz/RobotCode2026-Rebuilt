package frc.robot.CatzSubsystems.CatzBoba;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class BobaIOTalonFX extends GenericTalonFXIOReal<BobaIO.BobaIOInputs> implements BobaIO{
    public BobaIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
