package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class HoodIOSim extends GenericIOSim<HoodIO.HoodIOInputs> implements HoodIO{

    public HoodIOSim(Gains gains) {
        super(gains);
    }
}
