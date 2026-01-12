package frc.robot.CatzSubsystems.CatzTurret;

import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.Utilities.LoggedTunableNumber;

public class CatzTurret extends ServoMotorSubsystem{
    public static final CatzTurret Instance = new CatzTurret();

    private CatzTurret(){
        super(getIOInstance(TurretConstants.getIOConfig()), "CatzTurret", TurretConstants.TURRET_THRESHOLD);
    }
}
