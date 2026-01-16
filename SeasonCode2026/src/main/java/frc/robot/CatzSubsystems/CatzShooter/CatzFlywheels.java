package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;

public class CatzFlywheels extends FlywheelMotorSubsystem<FlywheelsIO, FlywheelsIO.FlywheelsIOInputs> {

    private static final FlywheelsIO io = getIOInstance();
    private static final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    private static FlywheelsIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new FlywheelsIOTalonFX(ClimbConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new FlywheelsIOSim();
                default:
                System.out.println("Roller Unconfigured");
                return new FlywheelsIOSim();
        }
    }

    public static final CatzFlywheels Instance = new CatzFlywheels();


    private CatzFlywheels() {
        super(io, inputs, "CatzFlywheels", FlywheelConstants.FLYWHEEL_THRESHOLD);
    }
}
