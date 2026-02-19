package frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOClawSim extends GenericIOSim<ClimbIOClaw.ClimbClawIOInputs> implements ClimbIOClaw {

    public ClimbIOClawSim(Gains gains) {
        super(gains);
    }

}
