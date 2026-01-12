package frc.robot.CatzSubsystems.CatzShooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface FlywheelsIO extends GenericMotorIO<FlywheelsIO.FlywheelsIOInputs>{
    @AutoLog
    public static class FlywheelsIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
