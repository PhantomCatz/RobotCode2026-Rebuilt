package frc.robot.CatzSubsystems.CatzTurret;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzSubsystems.CatzTurret.TurretIO.TurretIOInputs;
import frc.robot.Utilities.Setpoint;

public class CatzTurret extends ServoMotorSubsystem<TurretIO, TurretIO.TurretIOInputs>{
    private static final TurretIO io = getIOInstance();
    private static final TurretIOInputs inputs = new TurretIOInputsAutoLogged();
    private double commandedPos;

    public static final CatzTurret Instance = new CatzTurret();
    public enum ShooterState{
        HOME,
        TRACKING,
        MANUAL;
    }

    public ShooterState state = ShooterState.HOME;

    private CatzTurret(){
        super(io, inputs, "CatzTurret", TurretConstants.TURRET_THRESHOLD);
        setCurrentPosition(Angle.ofBaseUnits(0.0, Units.Degrees));
    }

    @Override
    public void periodic(){
        super.periodic();
        Logger.recordOutput("Turret Commanded Setpoint", setpoint.baseUnits / (2*Math.PI));


    }

    /**
     * Calculates the best turret angle setpoint to reach a target rotation
     * while respecting physical limits and minimizing movement
     * Returns null when no valid setpoint can be found.
     */
    public Setpoint calculateWrappedSetpoint(Angle target){
        double targetRads = target.in(Units.Radians);
        double minLegalRads = TurretConstants.TURRET_MIN.in(Units.Radians);
        double maxLegalRads = TurretConstants.TURRET_MAX.in(Units.Radians);

        targetRads = MathUtil.angleModulus(targetRads);
        Logger.recordOutput("Turret Target Position", targetRads / (2*Math.PI));

        return Setpoint.withMotionMagicSetpoint(Units.Radians.of(targetRads));
    }

    // public Command runthefreakingmotor() {
    //     return runOnce(() -> io.runMotor());
    // }

    private static TurretIO getIOInstance(){
        if (CatzConstants.TurretOn == false) {
            System.out.println("Turret Disabled by CatzConstants");
            return new TurretIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Turret Configured for Real");
                return new TurretIOTalonFX(TurretConstants.getIOConfig());
            case SIM:
                System.out.println("Turret Configured for Simulation");
                return new TurretIOSim();
            default:
                System.out.println("Turret Unconfigured");
            return new TurretIOSim();
        }
    }
}
