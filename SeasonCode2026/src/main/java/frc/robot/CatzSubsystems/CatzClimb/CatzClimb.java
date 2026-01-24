package frc.robot.CatzSubsystems.CatzClimb;


import org.littletonrobotics.junction.Logger;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimb extends ServoMotorSubsystem<ClimbIO, ClimbIO.ClimbIOInputs> {

    private static final ClimbIO io = getIOInstance();
    private static final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private static ClimbIO getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOTalonFX(ClimbConstants.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOSim();
                default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOSim();
        }
    }

    public static final CatzClimb Instance = new CatzClimb();

    private CatzClimb() {
        super(io, inputs, "CatzClimb", ClimbConstants.converter.toAngle(ClimbConstants.CLIMB_THRESHOLD));
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("Climb/TargetPosition", setpoint.baseUnits);
    }
}
