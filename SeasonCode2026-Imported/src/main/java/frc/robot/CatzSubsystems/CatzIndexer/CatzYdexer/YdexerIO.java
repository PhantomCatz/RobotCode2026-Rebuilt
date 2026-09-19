package frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface YdexerIO extends GenericMotorIO<YdexerIO.YdexerIOInputs>{
    @AutoLog
    public static class YdexerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
