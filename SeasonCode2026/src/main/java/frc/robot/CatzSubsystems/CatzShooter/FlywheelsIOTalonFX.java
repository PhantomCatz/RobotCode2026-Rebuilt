package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class FlywheelsIOTalonFX extends GenericTalonFXIOReal<FlywheelsIO.FlywheelsIOInputs> implements FlywheelsIO{
    public FlywheelsIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
