package frc.robot.CatzSubsystems.CatzIntakeRoller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface IntakeRollerIO extends GenericMotorIO<IntakeRollerIO.IntakeRollerIOInputs>{

    @AutoLog
    public static class IntakeRollerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
