package frc.robot.CatzSubsystems.CatzIndexer;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.CatzSubsystems.CatzClimb.ClimbIOSim;

public class CatzIndexer extends GenericMotorSubsystem<IndexerIO, IndexerIO.IndexerIOInputs>{

    private static final IndexerIO io = getIOInstance();
    private static final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private static IndexerIO getIOInstance() {
        if (CatzConstants.IndexerOn == false) {
            System.out.println("Indexer Disabled by CatzConstants");
            return new IndexerIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Indexer Configured for Real");
                return new IndexerIOTalonFX(IndexerConstants.getIOConfig());
            case SIM:
                System.out.println("Indexer Configured for Simulation");
                return new IndexerIOSim();
                default:
                System.out.println("Indexer Unconfigured");
                return new IndexerIOSim();
        }
    }

    private CatzIndexer() {
        super(io, inputs, "CatzIndexer");
    }

    public static final CatzIndexer Instance = new CatzIndexer();

}
