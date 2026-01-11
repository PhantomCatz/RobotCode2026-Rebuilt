package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;

public class CatzShooter extends FlywheelMotorSubsystem {

    private static final ShooterIO io = getIOInstance();

    public static final CatzShooter Instance = new CatzShooter();

    private CatzShooter() {
        super(io, "CatzShooter", 0.0); //TODO magic number!!z
    }

    static ShooterIO getIOInstance() {
        if (io != null) {
            return io;
        } else {
            switch (CatzConstants.hardwareMode) {
                case REAL:
                    System.out.println("Shooter Configured for Real");
                    return new ShooterIOTalonFX(ShooterConstants.getIOConfig());
                case SIM:
                    System.out.println("Shooter Configured for Simulation");
                    return new ShooterIOSim();
                default:
                    System.out.println("Shooter Unconfigured; defaulting to simulation");
                    return new ShooterIOSim();
            }
        }
    }
}
