package frc.robot.CatzSubsystems.CatzMantaArm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface MantaArmIO extends GenericMotorIO<MantaArmIO.MantaArmIOInputs>{

    @AutoLog
    public static class MantaArmIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
