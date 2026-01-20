package frc.robot.CatzSubsystems.CatzTurret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{
    private PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private PositionDutyCycle noFOC = new PositionDutyCycle(0.0).withUpdateFreqHz(0.0);
    public TurretIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
        System.out.println(" sfgjdofigjo idjfgoij sfoigj diofjg \n\n\n\n\n\n\n\n\n dpfgjodifjgos jfgoijsfg \n\n\n\n\n\n\n\n " + leaderTalon.getDeviceID());
    }

    // @Override
    // public void runMotor() {
    //     leaderTalon.set(0.3);
    // }

    // @Override
    // public void setPositionSetpoint(double targetRot){
    //     double targetRads = targetRot * 2*Math.PI;
    //     double robotAngularVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond;
    //     double feedforward = -TurretConstants.ROBOT_OMEGA_FEEDFORWARD * robotAngularVelocity;
    //     // System.out.println("ni hao: " + Angle.ofBaseUnits(targetRads, Units.Radians) + " roboAngularVafseiih " + robotAngularVelocity + " feedwoard ?" + feedforward);
    //     leaderTalon.setControl(requestGetter.getPositionRequest(Units.Radians.of(targetRads)));
    //         positionTorqueCurrentRequest.withPosition(Angle.ofBaseUnits(targetRads, Units.Radians))
    //                                     .withFeedForward(0.7);
    // }

    private double previousTime = 0.0;

    @Override
    public void setMotionMagicSetpoint(double targetRot){
        double currentTime = Timer.getFPGATimestamp();
        double robotAngularVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond / (2*Math.PI);
        double feedforward = TurretConstants.ROBOT_OMEGA_FEEDFORWARD * robotAngularVelocity;
            // System.out.println("ni hao: " + Angle.ofBaseUnits(targetRads, Units.Radians) + " roboAngularVafseiih " + robotAngularVelocity + " feedwoard ?" + feedforward);
        leaderTalon.setControl(new MotionMagicVoltage(targetRot).withFeedForward(feedforward));
    }
}
