package frc.robot.CatzSubsystems.CatzElevator;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class ElevatorIOTalonFX extends GenericTalonFXIOReal<ElevatorIO.ElevatorIOInputs> implements ElevatorIO{
    public ElevatorIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward;
        if(CatzElevator.Instance.getLatencyCompensatedPosition() > 0.28){
            feedforward = 0.0;
        }else{
            feedforward = -ElevatorConstants.GRAVITY_FEEDFORWARD* Math.sin(CatzElevator.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        }
        // Logger.recordOutput("Elevator Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -ElevatorConstants.GRAVITY_FEEDFORWARD * Math.sin(CatzElevator.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
