package frc.robot.CatzSubsystems.CatzSpindexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface SpindexerIO extends GenericMotorIO<SpindexerIO.IndexerIOInputs>{
    @AutoLog
    public static class IndexerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
