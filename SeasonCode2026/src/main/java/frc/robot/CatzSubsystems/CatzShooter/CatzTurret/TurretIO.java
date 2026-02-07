package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface TurretIO extends GenericMotorIO<TurretIO.TurretIOInputs>{
    @AutoLog
    public static class TurretIOInputs extends GenericMotorIO.MotorIOInputs{}
}
