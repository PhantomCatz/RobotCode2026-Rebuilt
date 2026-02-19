package frc.robot.CatzSubsystems.CatzClimb.CatzClimbTall;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ClimbIOTalonFXTall extends GenericTalonFXIOReal<ClimbIOTall.ClimbTallIOInputs> implements ClimbIOTall{
    public ClimbIOTalonFXTall(MotorIOTalonFXConfig config){
        super(config);
    }
}
