package frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzYdexer extends GenericMotorSubsystem<YdexerIO, YdexerIO.YdexerIOInputs>{
    private static final YdexerIO io = getIOInstance();
    private static final YdexerIOInputsAutoLogged inputs = new YdexerIOInputsAutoLogged();

    private static YdexerIO getIOInstance(){
        if(CatzConstants.YdexerOn == false){
            System.out.println("Ydexer Disabled by CatzConstants");
            return new YdexerIOSim(YdexerConstants.gains);
        }
        switch(CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Ydexer Configured for Real");
                return new YdexerIOTalonFX(YdexerConstants.getIOConfig(), true);
            case SIM:
                System.out.println("Ydexer Configured for Simulation");
                return new YdexerIOSim(YdexerConstants.gains);
            default:
                System.out.println("Ydexer Unconfigured");
                return new YdexerIOSim(YdexerConstants.gains);
        }
    }

    private CatzYdexer(){
        super(io, inputs, "CatzYdexer");
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = YdexerConstants.kP.get();
        double newV = YdexerConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }


    public static final CatzYdexer Instance = new CatzYdexer();
}
