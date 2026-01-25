package frc.robot.CatzSubsystems.CatzIntakeDeploy;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface IntakeDeployIO extends GenericMotorIO<IntakeDeployIO.IntakeDeployIOInputs>{

    @AutoLog
    public static class IntakeDeployIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
