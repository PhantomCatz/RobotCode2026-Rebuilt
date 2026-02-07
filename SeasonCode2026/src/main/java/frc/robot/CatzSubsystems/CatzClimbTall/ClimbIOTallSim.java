package frc.robot.CatzSubsystems.CatzClimbTall;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOTallSim extends GenericIOSim<ClimbIOTall.ClimbTallIOInputs> implements ClimbIOTall {

    public ClimbIOTallSim(Gains gains) {
        super(gains);
    }

}
