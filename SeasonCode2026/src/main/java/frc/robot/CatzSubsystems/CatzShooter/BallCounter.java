package frc.robot.CatzSubsystems.CatzShooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;

public class BallCounter extends SubsystemBase {

    double oldv = 0.0;
    int ballsshot = 0;
    boolean ballshot = false;
    @Override
    public void periodic() {

        double velocity = CatzFlywheels.Instance.i;
        while (CatzConstants.ShooterOn == true) {
            if (velocity >= oldv) {
                oldv = velocity;
                ballshot = false;
            } else if (velocity < oldv && ballshot == false) {
                ++ballsshot;
                boolean ballshot = true;;
                System.out.println("Balls shot:" + ballsshot);
            } else {
                oldv = velocity;
                ballshot = false;
            }
        }
    }
    
}
