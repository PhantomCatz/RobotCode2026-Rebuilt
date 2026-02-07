package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeRollerIOTalonFX extends GenericTalonFXIOReal<IntakeRollerIO.IntakeRollerIOInputs> implements IntakeRollerIO{
    public IntakeRollerIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

}
