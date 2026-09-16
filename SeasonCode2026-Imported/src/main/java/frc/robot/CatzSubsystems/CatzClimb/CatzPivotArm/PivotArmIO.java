package frc.robot.CatzSubsystems.CatzPivotArm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzAbstractions.io.GenericMotorIO;

public interface PivotArmIO extends GenericMotorIO<PivotArmIO.PivotArmIOInputs>{

    @AutoLog
    public static class PivotArmIOInputs extends GenericMotorIO.MotorIOInputs{

    }
}
