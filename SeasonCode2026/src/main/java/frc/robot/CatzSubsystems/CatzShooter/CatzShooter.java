package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;
import frc.robot.CatzAbstractions.io.GenericMotorIO;

public class CatzShooter extends FlywheelMotorSubsystem {

    private final static GenericMotorIO io = getIOInstance(ShooterConstants.getIOConfig());

    public static final CatzShooter Instance = new CatzShooter();

    private CatzShooter() {
        super(io, "CatzShooter", ShooterConstants.FLYWHEEL_THRESHOLD); // TODO magic number!!z
    }
}
