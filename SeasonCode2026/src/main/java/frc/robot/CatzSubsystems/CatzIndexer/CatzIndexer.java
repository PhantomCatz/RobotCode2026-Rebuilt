package frc.robot.CatzSubsystems.CatzIndexer;

import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzIndexer extends GenericMotorSubsystem{
    public static final CatzIndexer Instance = new CatzIndexer();

    private CatzIndexer(){
        super(getIOInstance(IndexerConstants.getIOConfig()), "CatzIndexer");
    }
}
