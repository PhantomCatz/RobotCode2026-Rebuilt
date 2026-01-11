package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;

public class CatzShooter extends FlywheelMotorSubsystem {

    public static final CatzShooter Instance = new CatzShooter();

    private CatzShooter() {
        super(getIOInstance(ShooterConstants.getIOConfig()), "CatzShooter", ShooterConstants.FLYWHEEL_THRESHOLD);
    }
}
