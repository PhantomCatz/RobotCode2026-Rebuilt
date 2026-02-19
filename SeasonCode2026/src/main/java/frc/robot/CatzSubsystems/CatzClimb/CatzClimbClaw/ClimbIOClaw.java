package frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIOClaw extends GenericMotorIO<ClimbIOClaw.ClimbClawIOInputs> {

    @AutoLog
    public static class ClimbClawIOInputs extends GenericMotorIO.MotorIOInputs{}
}
