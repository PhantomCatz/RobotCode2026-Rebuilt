package frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface SpindexerIO extends GenericMotorIO<SpindexerIO.SpindexerIOInputs>{
    @AutoLog
    public static class SpindexerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
