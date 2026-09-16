package frc.robot.CatzSubsystems.CatzPivotArm;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class PivotArmIOSim extends GenericIOSim<PivotArmIO.PivotArmIOInputs> implements PivotArmIO{
    public PivotArmIOSim(Gains gains){
        super(gains);
    }
}
