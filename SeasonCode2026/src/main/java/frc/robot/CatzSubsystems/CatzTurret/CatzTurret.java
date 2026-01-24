package frc.robot.CatzSubsystems.CatzTurret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzSubsystems.CatzTurret.TurretIO.TurretIOInputs;
import frc.robot.Utilities.Setpoint;

public class CatzTurret extends ServoMotorSubsystem<TurretIO, TurretIO.TurretIOInputs>{
    private static final TurretIO io = getIOInstance();
    private static final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

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

        Rotation2d rot = Rotation2d.fromRotations(getSetpoint().baseUnits);
        Pose2d pos = new Pose2d(2.0, 2.0, rot);
        Logger.recordOutput("Turret Angle", pos);
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
        double currentSetpointRads = targetRads;

        double bestSetpoint = currentSetpointRads;
        boolean foundValidOption = false;

        for(int i = -2; i <= TurretConstants.NUM_OF_FULL_ROT; i++){
            double potentialSetpoint = targetRads + (2.0 * Math.PI * i);

            // First check if this angle is physically possible for the hardware
            if (potentialSetpoint < minLegalRads || potentialSetpoint > maxLegalRads) {
                continue;
            }

            // If this is the first valid option we found or if it is closer
            // to our current position than the previous best option then pick it
            if (!foundValidOption) {
                bestSetpoint = potentialSetpoint;
                foundValidOption = true;
            } else if (Math.abs(currentSetpointRads - potentialSetpoint) < Math.abs(currentSetpointRads - bestSetpoint)) {
                bestSetpoint = potentialSetpoint;
            }
        }

        if(!foundValidOption){
            return null;
        }
        return Setpoint.withPositionSetpoint(Units.Radians.of(bestSetpoint));
    }

    private static TurretIO getIOInstance(){
        if (CatzConstants.TurretOn == false) {
            System.out.println("Turret Disabled by CatzConstants");
            return new TurretIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Roller Configured for Real");
                return new TurretIOTalonFX(TurretConstants.getIOConfig());
            case SIM:
                System.out.println("Roller Configured for Simulation");
                return new TurretIOSim();
            default:
                System.out.println("Roller Unconfigured");
            return new TurretIOSim();
        }
    }
}
