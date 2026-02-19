package frc.robot.CatzSubsystems.CatzClimb.CatzClimbShort;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOSim extends GenericIOSim<ClimbIO.ClimbIOInputs> implements ClimbIO {

    public ClimbIOSim(Gains gains) {
        super(gains);
    }

}
