package frc.robot.CatzSubsystems.CatzVdexer;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class VdexerIOTalonFX extends GenericTalonFXIOReal<VdexerIO.VdexerIOInputs> implements VdexerIO{
    public VdexerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
