package frc.robot.CatzSubsystems.MantaPivot;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class MantaPivotIOSim extends GenericIOSim<MantaPivotIO.MantaPivotIOInputs> implements MantaPivotIO{
    public MantaPivotIOSim(Gains gains){
        super(gains);
    }
}
