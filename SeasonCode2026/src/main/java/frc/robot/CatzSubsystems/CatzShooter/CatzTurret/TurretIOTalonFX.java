package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{

    public TurretIOTalonFX(MotorIOTalonFXConfig config, boolean requiresFastUpdate){
        super(config, requiresFastUpdate);
    }

    @Override
    public void setMotionMagicSetpoint(double targetRot){
        double curAngularVel = CatzRobotTracker.getInstance().getRobotChassisSpeeds().omegaRadiansPerSecond;
        double velFeedforward = -TurretConstants.omegaFF.get() * curAngularVel;
        leaderTalon.setControl(new MotionMagicVoltage(targetRot).withFeedForward(velFeedforward));
    }
}
