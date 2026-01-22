package frc.robot.CatzSubsystems.CatzSpindexer;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class SpindexerIOTalonFX extends GenericTalonFXIOReal<SpindexerIO.IndexerIOInputs> implements SpindexerIO{
    public SpindexerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
