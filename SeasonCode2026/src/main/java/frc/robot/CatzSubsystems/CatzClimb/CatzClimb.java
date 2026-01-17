package frc.robot.CatzSubsystems.CatzClimb;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimb extends ServoMotorSubsystem<ClimbIO, ClimbIO.ClimbIOInputs> {

    private static final ClimbIO io = getIOInstance();
    private static final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private static ClimbIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new ClimbIOTalonFX(ClimbConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new ClimbIOSim(ClimbConstants.gains);
                default:
                System.out.println("Roller Unconfigured");
                return new ClimbIOSim(ClimbConstants.gains);
        }
    }

    public static final CatzClimb Instance = new CatzClimb();

    private CatzClimb() {
        super(io, inputs, "CatzClimb", ClimbConstants.converter.toAngle(ClimbConstants.CLIMB_THRESHOLD));
    }

}
