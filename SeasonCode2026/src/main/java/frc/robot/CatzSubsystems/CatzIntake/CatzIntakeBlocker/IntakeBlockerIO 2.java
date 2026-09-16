package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeBlocker;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface IntakeBlockerIO extends GenericMotorIO<IntakeBlockerIO.IntakeBlockerIOInputs>{

    @AutoLog
    public static class IntakeBlockerIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
