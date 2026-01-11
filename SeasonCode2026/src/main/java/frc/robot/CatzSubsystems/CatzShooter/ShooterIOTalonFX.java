package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ShooterIOTalonFX extends GenericTalonFXIOReal implements ShooterIO{

    //TODO are these different types of IO classes needed?
    public ShooterIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
