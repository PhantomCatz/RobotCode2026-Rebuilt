package frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzSpindexer extends GenericMotorSubsystem<SpindexerIO, SpindexerIO.SpindexerIOInputs>{
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
                return new SpindexerIOTalonFX(SpindexerConstants.getIOConfig(), true);
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

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = SpindexerConstants.kP.get();
        double newV = SpindexerConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }


    public static final CatzSpindexer Instance = new CatzSpindexer();
}
