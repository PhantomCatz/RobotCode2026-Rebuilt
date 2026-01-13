package frc.robot.CatzSubsystems.CatzIntake;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeIOTalonFX extends GenericTalonFXIOReal<IntakeIO.IntakeIOInputs> implements IntakeIO{
    public IntakeIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
}
