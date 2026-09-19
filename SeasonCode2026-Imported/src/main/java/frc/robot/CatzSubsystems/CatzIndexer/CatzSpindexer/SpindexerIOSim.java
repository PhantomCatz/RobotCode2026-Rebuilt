package frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class SpindexerIOSim extends GenericIOSim<SpindexerIO.SpindexerIOInputs> implements SpindexerIO{

    public SpindexerIOSim(Gains gains) {
        super(gains);
    }
}
