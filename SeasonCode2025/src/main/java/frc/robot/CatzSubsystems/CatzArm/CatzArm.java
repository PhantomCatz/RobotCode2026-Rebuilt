package frc.robot.CatzSubsystems.CatzArm;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import lombok.RequiredArgsConstructor;

public class CatzArm extends SubsystemBase {
    public static final CatzArm Instance = new CatzArm();

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private static ArmPosition targetPosition = ArmPosition.STOW;

    @RequiredArgsConstructor
    public enum ArmPosition { //In degrees
        STOW(() -> 0.0),
        NULL(() -> 0.0),
        UP(() -> 50); // TODO

        private final DoubleSupplier motionType;
        private double getTargetAngle() {
            return motionType.getAsDouble();
        }
    }

    private CatzArm() {
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
                    System.out.println("Arm Configured for Simulation");
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

        if (DriverStation.isDisabled()) {
            io.setPercentOutput(0.0);
            targetPosition = ArmPosition.NULL;
        } else {
            if(targetPosition != ArmPosition.NULL) {
                io.runSetpointUp(targetPosition.getTargetAngle(), 0.0);
            }
        }
    }

    public Command armStow() {
        return runOnce(() -> setArmPos(ArmPosition.STOW));
    }

    public Command armUp() {
        return runOnce(() -> setArmPos(ArmPosition.UP));
    }

    public void setArmPos(ArmPosition target) {
        System.out.println("set pose");
        targetPosition = target;
    }
}
