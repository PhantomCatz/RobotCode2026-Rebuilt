package frc.robot.CatzSubsystems.CatzClimb.CatzClimbTall;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIOTall extends GenericMotorIO<ClimbIOTall.ClimbTallIOInputs> {

    @AutoLog
    public static class ClimbTallIOInputs extends GenericMotorIO.MotorIOInputs{}
}
