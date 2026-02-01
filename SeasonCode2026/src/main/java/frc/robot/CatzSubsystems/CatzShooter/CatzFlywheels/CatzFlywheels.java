package frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;

public class CatzFlywheels extends FlywheelMotorSubsystem<FlywheelsIO, FlywheelsIO.FlywheelsIOInputs> {

    private static final FlywheelsIO io = getIOInstance();
    private static final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    private static FlywheelsIO getIOInstance() {
        if (CatzConstants.ShooterOn == false) {
            System.out.println("Shooter Disabled by CatzConstants");
            return new FlywheelsIOSim(FlywheelConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new FlywheelsIOTalonFX(FlywheelConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new FlywheelsIOSim(FlywheelConstants.gains);
                default:
                System.out.println("Roller Unconfigured");
                return new FlywheelsIOSim(FlywheelConstants.gains);
        }
    }

    public static final CatzFlywheels Instance = new CatzFlywheels();

    double prevP = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = FlywheelConstants.kP.get();
        if(newP != prevP){
            setGainsP(newP);
            prevP = newP;
        }
    }

    private CatzFlywheels() {
        super(io, inputs, "CatzFlywheels", FlywheelConstants.FLYWHEEL_THRESHOLD);
    }
}
