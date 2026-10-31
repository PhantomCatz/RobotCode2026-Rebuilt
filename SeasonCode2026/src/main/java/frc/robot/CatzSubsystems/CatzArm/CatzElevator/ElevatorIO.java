package frc.robot.CatzSubsystems.CatzArm.CatzElevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface ElevatorIO extends GenericMotorIO<ElevatorIO.ElevatorIOInputs>{
    @AutoLog
    public static class ElevatorIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
