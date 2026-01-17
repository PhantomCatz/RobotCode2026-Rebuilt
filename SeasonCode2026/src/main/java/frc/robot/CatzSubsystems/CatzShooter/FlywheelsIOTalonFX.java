package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class FlywheelsIOTalonFX extends GenericTalonFXIOReal<FlywheelsIO.FlywheelsIOInputs> implements FlywheelsIO{
    public FlywheelsIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
        System.out.println("leader talon::::\n\n\n\n\n:: ;;;;;; ;;;;;;;; " + leaderTalon.getDeviceID());
        System.out.println("follower talon::\n\n\n\n\n\n:::: ;;;;;; ;;;;;;;; " + followerTalons[0].getDeviceID());

    }
}
