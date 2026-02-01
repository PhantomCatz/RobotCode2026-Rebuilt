package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{

    public TurretIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }
    double prevAngularVel = 0.0;
    double prevTimestamp = Timer.getFPGATimestamp();
    private final double TO_ROT = 1.0 / (2*Math.PI);
    @Override
    public void setMotionMagicSetpoint(double targetRot){
        double curTimestamp = Timer.getFPGATimestamp();
        double curAngularVel = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond;
        double timePassed = curTimestamp-prevTimestamp;
        double velDiff = curAngularVel-prevAngularVel;
        double accFeedforward = TurretConstants.ROBOT_ACCELERATION_FEEDFORWARD * velDiff/timePassed;
        double velFeedforward = -TurretConstants.omegaFF.get() * curAngularVel;
        prevAngularVel = curAngularVel;
        prevTimestamp = curTimestamp;
        leaderTalon.setControl(new MotionMagicVoltage(targetRot).withFeedForward(accFeedforward+velFeedforward));
    }
}
