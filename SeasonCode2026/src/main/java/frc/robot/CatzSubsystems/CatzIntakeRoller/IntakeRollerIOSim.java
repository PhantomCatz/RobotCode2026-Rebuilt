package frc.robot.CatzSubsystems.CatzIntakeRoller;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeRollerIOSim extends GenericIOSim<IntakeRollerIO.IntakeRollerIOInputs> implements IntakeRollerIO{
    public IntakeRollerIOSim(Gains gains){
        super(gains);
    }    
}
