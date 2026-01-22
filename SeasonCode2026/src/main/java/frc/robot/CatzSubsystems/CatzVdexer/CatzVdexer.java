package frc.robot.CatzSubsystems.CatzVdexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzVdexer extends GenericMotorSubsystem<VdexerIO, VdexerIO.VdexerIOInputs>{

    private static final VdexerIO io = getIOInstance();
    private static final VdexerIOInputsAutoLogged inputs = new VdexerIOInputsAutoLogged();

    private static VdexerIO getIOInstance() {
        if (CatzConstants.IndexerOn == false) {
            System.out.println("Indexer Disabled by CatzConstants");
            return new VdexerIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Indexer Configured for Real");
                return new VdexerIOTalonFX(VdexerConstants.getIOConfig());
            case SIM:
                System.out.println("Indexer Configured for Simulation");
                return new VdexerIOSim();
                default:
                System.out.println("Indexer Unconfigured");
                return new VdexerIOSim();
        }
    }

    private CatzVdexer() {
        super(io, inputs, "CatzIndexer");
    }

    public static final CatzVdexer Instance = new CatzVdexer();

}
