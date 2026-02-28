package frc.robot.CatzSubsystems.CatzClimbElevator;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ClimbIOTalonFXElevator extends GenericTalonFXIOReal<ClimbIOElevator.ClimbElevatorIOInputs> implements ClimbIOElevator{
    public ClimbIOTalonFXElevator(MotorIOTalonFXConfig config){
        super(config);
    }
}
