package frc.robot.CatzSubsystems.CatzArm.CatzElevator;


import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzArm.CatzMantaPivot.CatzMantaPivot;

public class ElevatorIOTalonFX extends GenericTalonFXIOReal<ElevatorIO.ElevatorIOInputs> implements ElevatorIO{
    public ElevatorIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        // Make feedforward depending on angle of pivot
        double feedforward = ElevatorConstants.GRAVITY_FEEDFORWARD * Math.sin(Units.rotationsToRadians(CatzMantaPivot.Instance.getLatencyCompensatedPosition()));
        System.out.println("feedforward"+feedforward);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }
}
