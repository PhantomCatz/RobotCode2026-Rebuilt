package frc.robot.CatzSubsystems.CatzArm;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

public class CatzArm extends SubsystemBase {
    public static final CatzArm Instance = new CatzArm();

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public CatzArm() {
        if(isArmDisabled) {
            io = new ArmIONull();
            System.out.println("Arm Unconfigured");
        } else {
            switch (CatzConstants.hardwareMode) {
                case REAL:
                    io = new ArmIOReal();
                    System.out.println("Arm Configured for Real");
                break;
                case REPLAY:
                    io = new ArmIOReal() {};
                    System.out.println("Arm Configured for Replayed simulation");
                break;
                case SIM:
                    io = new ArmIOSim();
                    System.out.println("Elevator Configured for Simulation");
                break;
                default:
                    io = new ArmIONull();
                    System.out.println("Arm Unconfigured");
                break;
            }
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RealInputs/Arm", inputs);
    }
}
