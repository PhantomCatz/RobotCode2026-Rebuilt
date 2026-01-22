package frc.robot.CatzSubsystems.CatzVdexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface VdexerIO extends GenericMotorIO<VdexerIO.VdexerIOInputs>{
    @AutoLog
    public static class VdexerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
