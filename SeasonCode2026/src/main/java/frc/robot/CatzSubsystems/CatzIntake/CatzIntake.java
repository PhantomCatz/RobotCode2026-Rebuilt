package frc.robot.CatzSubsystems.CatzIntake;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzIntake extends GenericMotorSubsystem {

    private static final IntakeIO io = getIOInstance();

    public static final CatzIntake Instance = new CatzIntake();

    private CatzShooter() {
        super()
    }

    static IntakeIO getIOInstance(){
        if(io != null){
            return io;
        }else{
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
