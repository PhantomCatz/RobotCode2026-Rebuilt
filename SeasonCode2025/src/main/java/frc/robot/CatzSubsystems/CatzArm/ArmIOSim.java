package frc.robot.CatzSubsystems.CatzArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class ArmIOSim implements ArmIO {
    private final DCMotor m_armGearbox = DCMotor.getKrakenX60Foc(1); // TODO motor type
    private double targetDegreesFinalShaft;
    private final int ARM_INDEX = 0;

    private PIDController simPIDController = new PIDController(0.1, 0.0, 0.0);

    private final SingleJointedArmSim m_armSim =
        new SingleJointedArmSim(
            m_armGearbox, 
            ArmConstants.ARM_GEAR_REDUCTION,
            ArmConstants.ARM_JKG_SQUARED,
            Units.inchesToMeters(ArmConstants.ARM_LENGTH_INCHES),
            Units.degreesToRadians(ArmConstants.ARM_MIN_DEGREES),
            Units.degreesToRadians(ArmConstants.ARM_MAX_DEGREES),
            true,
            Units.degreesToRadians(ArmConstants.ARM_INITIAL_DEGREES),
            0.01,
            0.0);
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionDegreesFinalShaft = Units.radiansToDegrees(m_armSim.getAngleRads());

        double setVoltage = simPIDController.calculate(inputs.positionDegreesFinalShaft, targetDegreesFinalShaft) * 12.0;
        m_armSim.setInputVoltage(setVoltage);
        m_armSim.update(0.02);

        Logger.recordOutput("Arm/Sim target degrees", targetDegreesFinalShaft);
        Robot.setSimPose(ARM_INDEX, new Pose3d(ArmConstants.ARM_SIM_OFFSET, new Rotation3d(0, Units.degreesToRadians(inputs.positionDegreesFinalShaft), 0)));
    }

    @Override
    public void runSetpointUp(double setpointDegrees) {
        System.out.println("setpoint up"+setpointDegrees);
        targetDegreesFinalShaft = setpointDegrees;
    }

    @Override
    public void runSetpointDown(double setpointDegrees) {
        targetDegreesFinalShaft = setpointDegrees;
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        m_armSim.setInputVoltage(percentOutput * 12.0);
    }
}
