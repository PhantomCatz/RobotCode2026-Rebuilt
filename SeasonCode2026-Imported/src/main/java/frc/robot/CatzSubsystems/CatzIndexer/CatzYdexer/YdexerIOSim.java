package frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class YdexerIOSim extends GenericIOSim<YdexerIO.YdexerIOInputs> implements YdexerIO{

    public YdexerIOSim(Gains gains) {
        super(gains);
    }
}
