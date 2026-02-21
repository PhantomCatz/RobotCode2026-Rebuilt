package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ClimbIOTalonFXElevator extends GenericTalonFXIOReal<ClimbIOElevator.ClimbTallIOInputs> implements ClimbIOElevator{
    public ClimbIOTalonFXElevator(MotorIOTalonFXConfig config){
        super(config);
    }
}
