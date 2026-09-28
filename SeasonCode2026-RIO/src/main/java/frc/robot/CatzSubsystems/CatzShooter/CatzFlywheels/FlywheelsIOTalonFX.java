package frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class FlywheelsIOTalonFX extends GenericTalonFXIOReal<FlywheelsIO.FlywheelsIOInputs> implements FlywheelsIO{
    public FlywheelsIOTalonFX(MotorIOTalonFXConfig config, boolean requiresFastUpdate){
        super(config, requiresFastUpdate);
    }
}
