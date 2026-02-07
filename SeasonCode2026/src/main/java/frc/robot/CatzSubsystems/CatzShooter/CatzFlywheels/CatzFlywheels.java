package frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;

import edu.wpi.first.units.Units;
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
                return new FlywheelsIOTalonFX(FlywheelConstants.getIOConfig(), true);
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
double prevV = 0.0;

public double i = inputs.velocityRPS;
double oldv = 0.0;
int ballsshot = 0;
boolean wasballshot = false;

    @Override
    public void periodic(){
        super.periodic();

        double newP = FlywheelConstants.kP.get();
        double newV = FlywheelConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
        
        double velocity = CatzFlywheels.Instance.getVelocity().in(Units.RotationsPerSecond);
        double epsilon = (velocity - (velocity * 0.04));
        if (spunUp()) {
            if (velocity >= oldv) {
                wasballshot = false;
            } else if (velocity < oldv && wasballshot == false) {
                if (velocity < epsilon) {
                    ++ballsshot;
                    wasballshot = true;
                    System.out.println("Balls shot:" + ballsshot);
                } else {
                    wasballshot = false;
                }

            }
            oldv = velocity;
        }

    }

    private CatzFlywheels() {
        super(io, inputs, "CatzFlywheels", FlywheelConstants.FLYWHEEL_THRESHOLD);
    }

}
