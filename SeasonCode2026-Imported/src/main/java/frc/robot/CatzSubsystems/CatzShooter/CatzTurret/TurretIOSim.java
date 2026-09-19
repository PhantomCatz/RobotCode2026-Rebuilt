package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class TurretIOSim extends GenericIOSim<TurretIO.TurretIOInputs> implements TurretIO{
    public TurretIOSim(Gains gains){
        super(gains);
    }
}
