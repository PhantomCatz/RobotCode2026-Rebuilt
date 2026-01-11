package frc.robot.CatzSubsystems.CatzShooter;

import frc.robot.CatzAbstractions.Bases.FlywheelMotorSubsystem;

public class CatzShooter extends FlywheelMotorSubsystem {

    private static final ShooterIO io = getIOInstance();

    public static final CatzShooter Instance = new CatzShooter();

    private CatzShooter() {

    }

    static ShooterIO getIOInstance(){
        if(io != null){
            return io;
        }else{
            switch(C)
        }
    }
}
