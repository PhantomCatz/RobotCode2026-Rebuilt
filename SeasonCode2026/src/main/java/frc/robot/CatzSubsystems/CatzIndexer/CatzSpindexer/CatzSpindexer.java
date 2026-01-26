package frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzSpindexer extends GenericMotorSubsystem{
    private static final SpindexerIO io = getIOInstance();
    private static final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private static SpindexerIO getIOInstance(){
        if(CatzConstants.SpindexerOn == false){
            System.out.println("Spindexer Disabled by CatzConstants");
            return new SpindexerIOSim(SpindexerConstants.gains);
        }
        switch(CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Spindexer Configured for Real");
                return new SpindexerIOTalonFX(SpindexerConstants.getIOConfig());
            case SIM:
                System.out.println("Spindexer Configured for Simulation");
                return new SpindexerIOSim(SpindexerConstants.gains);
            default:
                System.out.println("Spindexer Unconfigured");
                return new SpindexerIOSim(SpindexerConstants.gains);
        }
    }

    private CatzSpindexer(){
        super(io, inputs, "CatzSpindexer");
    }

    public static final CatzSpindexer Instance = new CatzSpindexer();
}
