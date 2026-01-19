package frc.robot.CatzSubsystems.CatzIntake;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeIOSim extends GenericIOSim<IntakeIO.IntakeIOInputs> implements IntakeIO{
    public IntakeIOSim(Gains gains) {
        super(gains);
    }
}
