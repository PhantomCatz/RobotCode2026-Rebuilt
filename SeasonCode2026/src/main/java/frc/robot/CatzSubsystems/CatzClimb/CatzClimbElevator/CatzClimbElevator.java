package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimbElevator extends ServoMotorSubsystem<ClimbIOElevator, ClimbIOElevator.ClimbElevatorIOInputs> {

    private static final ClimbIOElevator io = getIOInstance();
    private static final ClimbElevatorIOInputsAutoLogged inputs = new ClimbElevatorIOInputsAutoLogged();

    private static ClimbIOElevator getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOElevatorSim(ClimbConstantsElevator.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOTalonFXElevator(ClimbConstantsElevator.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOElevatorSim(ClimbConstantsElevator.gains);
            default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOElevatorSim(ClimbConstantsElevator.gains);
        }
    }

    public static final CatzClimbElevator Instance = new CatzClimbElevator();

    private CatzClimbElevator() {
        super(io, inputs, "CatzClimb", ClimbConstantsElevator.converter.toAngle(ClimbConstantsElevator.CLIMB_THRESHOLD));
    }

}
