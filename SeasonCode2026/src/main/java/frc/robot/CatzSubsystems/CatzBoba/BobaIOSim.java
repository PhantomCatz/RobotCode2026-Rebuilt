package frc.robot.CatzSubsystems.CatzBoba;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class BobaIOSim extends GenericIOSim<BobaIO.BobaIOInputs> implements BobaIO {

    public BobaIOSim(Gains gains) {
        super(gains);
    }

}
