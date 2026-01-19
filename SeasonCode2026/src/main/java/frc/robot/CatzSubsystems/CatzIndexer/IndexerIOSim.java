package frc.robot.CatzSubsystems.CatzIndexer;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class IndexerIOSim extends GenericIOSim<IndexerIO.IndexerIOInputs> implements IndexerIO{

    public IndexerIOSim(Gains gains) {
        super(gains);
    }
}
