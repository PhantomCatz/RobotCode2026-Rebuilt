package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

    public static final Music Instance = new Music();

    private final Orchestra orchestra = new Orchestra();
    private final TalonFX musicMotor = new TalonFX(1);

    private Music() {
        System.out.println("Music Subsystem Initializing");
        orchestra.addInstrument(musicMotor);
    }

    public Command playSongCommand(String songFileName) {
        return Commands.runOnce(() -> {
            System.out.println("Loading and playing " + songFileName);
            orchestra.loadMusic(songFileName);
            orchestra.play();
        }, this).ignoringDisable(true);
    }

    public Command stopMusicCommand() {
        return Commands.runOnce(() -> {
            orchestra.stop();
        }, this).ignoringDisable(true);
    }
}
