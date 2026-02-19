package frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimbClaw extends ServoMotorSubsystem<ClimbIOClaw, ClimbIOClaw.ClimbClawIOInputs> {

    private static final ClimbIOClaw io = getIOInstance();
    private static final ClimbClawIOInputsAutoLogged inputs = new ClimbClawIOInputsAutoLogged();

    private static ClimbIOClaw getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOClawSim(ClimbConstantsClaw.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOSparkmaxClaw(ClimbConstantsClaw.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOClawSim(ClimbConstantsClaw.gains);
                default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOClawSim(ClimbConstantsClaw.gains);
        }
    }

    public static final CatzClimbClaw Instance = new CatzClimbClaw();

    private CatzClimbClaw() {
        super(io, inputs, "CatzClimb", ClimbConstantsClaw.converter.toAngle(ClimbConstantsClaw.CLIMB_THRESHOLD));
    }

}
