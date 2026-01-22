package frc.robot.CatzSubsystems.CatzClimbTall;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIOTall extends GenericMotorIO<ClimbIOTall.ClimbIOInputs> {

    @AutoLog
    public static class ClimbIOInputs extends GenericMotorIO.MotorIOInputs{}
}
