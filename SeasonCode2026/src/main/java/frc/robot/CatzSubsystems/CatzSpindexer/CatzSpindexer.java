package frc.robot.CatzSubsystems.CatzSpindexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzSpindexer extends GenericMotorSubsystem<SpindexerIO, SpindexerIO.IndexerIOInputs>{

    private static final SpindexerIO io = getIOInstance();
    private static final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private static SpindexerIO getIOInstance() {
        if (CatzConstants.IndexerOn == false) {
            System.out.println("Indexer Disabled by CatzConstants");
            return new SpindexerIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Indexer Configured for Real");
                return new SpindexerIOTalonFX(SpindexerConstants.getIOConfig());
            case SIM:
                System.out.println("Indexer Configured for Simulation");
                return new SpindexerIOSim();
                default:
                System.out.println("Indexer Unconfigured");
                return new SpindexerIOSim();
        }
    }

    private CatzSpindexer() {
        super(io, inputs, "CatzIndexer");
    }

    public static final CatzSpindexer Instance = new CatzSpindexer();

}
