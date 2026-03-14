package frc.robot.CatzSubsystems.CatzBoba;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface BobaIO extends GenericMotorIO<BobaIO.BobaIOInputs> {

    @AutoLog
    public static class BobaIOInputs extends GenericMotorIO.MotorIOInputs{}
}
