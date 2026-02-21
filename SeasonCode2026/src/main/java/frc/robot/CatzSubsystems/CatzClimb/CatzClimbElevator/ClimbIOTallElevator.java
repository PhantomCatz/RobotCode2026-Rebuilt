package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOTallElevator extends GenericIOSim<ClimbIOElevator.ClimbTallIOInputs> implements ClimbIOElevator {

    public ClimbIOTallElevator(Gains gains) {
        super(gains);
    }

}
