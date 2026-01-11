package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ShooterIOTalonFX extends GenericTalonFXIOReal implements ShooterIO{
    public ShooterIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
