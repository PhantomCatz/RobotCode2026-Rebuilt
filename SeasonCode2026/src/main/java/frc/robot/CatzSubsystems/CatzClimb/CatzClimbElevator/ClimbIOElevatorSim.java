package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class ClimbIOElevatorSim extends GenericIOSim<ClimbIOElevator.ClimbElevatorIOInputs> implements ClimbIOElevator {

    public ClimbIOElevatorSim(Gains gains) {
        super(gains);
    }

}
