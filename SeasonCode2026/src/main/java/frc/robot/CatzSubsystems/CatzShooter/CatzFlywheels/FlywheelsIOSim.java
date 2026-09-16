package frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.Utilities.MotorUtil.Gains;

public class FlywheelsIOSim extends GenericIOSim<FlywheelsIO.FlywheelsIOInputs> implements FlywheelsIO{
    public FlywheelsIOSim(Gains gains) {
        super(gains);
    }
}
