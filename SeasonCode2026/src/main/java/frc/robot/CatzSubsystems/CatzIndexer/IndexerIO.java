package frc.robot.CatzSubsystems.CatzIndexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface IndexerIO extends GenericMotorIO<IndexerIO.IndexerIOInputs>{
    @AutoLog
    public static class IndexerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
