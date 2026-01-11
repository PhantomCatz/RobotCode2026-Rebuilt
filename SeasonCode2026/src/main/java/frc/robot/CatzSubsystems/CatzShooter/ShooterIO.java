package frc.robot.CatzSubsystems.CatzShooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ShooterIO extends GenericMotorIO{

    //TODO is this ShooterIO interface needed? Don't all motors have the same IO?
    @AutoLog
    public static class ShooterIOInputs{

    }
}
