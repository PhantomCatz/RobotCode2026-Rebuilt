package frc.robot.CatzSubsystems.CatzMantaPivot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface MantaPivotIO extends GenericMotorIO<MantaPivotIO.MantaPivotIOInputs>{

    @AutoLog
    public static class MantaPivotIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
