package frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class YdexerIOTalonFX extends GenericTalonFXIOReal<YdexerIO.YdexerIOInputs> implements YdexerIO{
    public YdexerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
