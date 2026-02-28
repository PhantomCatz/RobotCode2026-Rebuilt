package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeDeployIOTalonFX extends GenericTalonFXIOReal<IntakeDeployIO.IntakeDeployIOInputs> implements IntakeDeployIO{
    public IntakeDeployIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        //TODO do not apply feed forward when intake is down to save some battery.
        double feedforward = -IntakeDeployConstants.kG.get() * Math.sin(CatzIntakeDeploy.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        Logger.recordOutput("Intake Deploy Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -IntakeDeployConstants.kG.get() * Math.sin(CatzIntakeDeploy.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
