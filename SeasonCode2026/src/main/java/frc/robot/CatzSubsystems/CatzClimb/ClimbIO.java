package frc.robot.CatzSubsystems.CatzClimb;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIO extends GenericMotorIO<ClimbIO.ClimbIOInputs>{

    @AutoLog
    public static class ClimbIOInputs extends GenericMotorIO.MotorIOInputs{}
}
