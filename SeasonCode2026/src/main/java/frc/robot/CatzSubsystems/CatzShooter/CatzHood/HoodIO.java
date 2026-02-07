package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface HoodIO extends GenericMotorIO<HoodIO.HoodIOInputs>{
    @AutoLog
    public static class HoodIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
