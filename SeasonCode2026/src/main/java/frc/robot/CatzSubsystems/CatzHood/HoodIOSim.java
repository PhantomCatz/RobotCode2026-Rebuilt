package frc.robot.CatzSubsystems.CatzHood;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class HoodIOSim extends GenericIOSim<HoodIO.HoodIOInputs> implements HoodIO{

    public HoodIOSim(Gains gains) {
        super(gains);
    }
}
