package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIOElevator extends GenericMotorIO<ClimbIOElevator.ClimbTallIOInputs> {

    @AutoLog
    public static class ClimbTallIOInputs extends GenericMotorIO.MotorIOInputs{}
}
