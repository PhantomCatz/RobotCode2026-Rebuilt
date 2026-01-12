package frc.robot.CatzSubsystems.CatzIndexer;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IndexerIOTalonFX extends GenericTalonFXIOReal<IndexerIO.IndexerIOInputs> implements IndexerIO{
    public IndexerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
