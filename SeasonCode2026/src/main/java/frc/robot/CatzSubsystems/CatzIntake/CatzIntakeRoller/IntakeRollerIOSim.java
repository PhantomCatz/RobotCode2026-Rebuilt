package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;

import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Utilities.MotorUtil.Gains;

public class IntakeRollerIOSim extends GenericIOSim<IntakeRollerIO.IntakeRollerIOInputs> implements IntakeRollerIO{

    private final IntakeSimulation intakeSimulation;

    public IntakeRollerIOSim(Gains gains) {
        super(gains);
        this.intakeSimulation =  IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            CatzDrivetrain.driveSimulationInstance.getDriveTrainSimulation(),
            // Width of the intake
            Meters.of(0.7),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 20 fuel
            40);
    }


    @Override
    public void setDutyCycleSetpoint(double voltage) {
        if(voltage != 0.0) {
            intakeSimulation.startIntake();
            System.out.println("start");
        } else {
            intakeSimulation.stopIntake();
        }
    }

}
