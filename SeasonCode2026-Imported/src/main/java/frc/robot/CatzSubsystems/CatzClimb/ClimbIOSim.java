package frc.robot.CatzSubsystems.CatzClimb;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOSim extends GenericIOSim<ClimbIO.ClimbIOInputs> implements ClimbIO {

    public ClimbIOSim(Gains gains) {
        super(gains);
    }

}
