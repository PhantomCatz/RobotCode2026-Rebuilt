package frc.robot.CatzSubsystems.CatzClimb;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ClimbIOTalonFX extends GenericTalonFXIOReal<ClimbIO.ClimbIOInputs> implements ClimbIO{
    public ClimbIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
        System.out.println(leaderTalon.getDeviceID());
    }
}
