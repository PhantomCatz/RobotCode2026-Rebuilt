package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class HoodIOTalonFX extends GenericTalonFXIOReal<HoodIO.HoodIOInputs> implements HoodIO{
    public HoodIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
