package frc.robot.CatzSubsystems.CatzIntake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface IntakeIO extends GenericMotorIO<IntakeIO.IntakeIOInputs>{

    @AutoLog
    public static class IntakeIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
