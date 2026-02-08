package frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class SpindexerIOTalonFX extends GenericTalonFXIOReal<SpindexerIO.SpindexerIOInputs> implements SpindexerIO{
    public SpindexerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
