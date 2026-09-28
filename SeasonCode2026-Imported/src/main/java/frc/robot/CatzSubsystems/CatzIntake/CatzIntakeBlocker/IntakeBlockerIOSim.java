package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeBlocker;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeBlockerIOSim extends GenericIOSim<IntakeBlockerIO.IntakeBlockerIOInputs> implements IntakeBlockerIO{
    public IntakeBlockerIOSim(Gains gains){
        super(gains);
    }
}
