package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;
import frc.robot.Utilities.Setpoint;

public class CatzShooter extends FlywheelMotorSubsystem {

    public static final CatzShooter Instance = new CatzShooter();

    private CatzShooter() {
        super(getIOInstance(ShooterConstants.getIOConfig()), "CatzShooter", ShooterConstants.FLYWHEEL_THRESHOLD);
    }

    public Setpoint getTunableSetpoint(){
        return Setpoint.withVelocitySetpoint(ShooterConstants.SHOOTING_RPS_TUNABLE.get());
    }
}
