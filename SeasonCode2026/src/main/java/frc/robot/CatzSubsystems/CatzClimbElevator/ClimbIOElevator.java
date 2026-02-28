package frc.robot.CatzSubsystems.CatzClimbElevator;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ClimbIOElevator extends GenericMotorIO<ClimbIOElevator.ClimbElevatorIOInputs> {

    @AutoLog
    public static class ClimbElevatorIOInputs extends GenericMotorIO.MotorIOInputs{}
}
