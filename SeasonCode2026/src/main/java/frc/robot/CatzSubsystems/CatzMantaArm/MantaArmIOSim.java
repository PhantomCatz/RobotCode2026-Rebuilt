package frc.robot.CatzSubsystems.CatzMantaArm;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class MantaArmIOSim extends GenericIOSim<MantaArmIO.MantaArmIOInputs> implements MantaArmIO{
    public MantaArmIOSim(Gains gains){
        super(gains);
    }
}
