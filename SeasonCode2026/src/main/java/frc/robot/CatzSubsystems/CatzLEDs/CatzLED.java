package frc.robot.CatzSubsystems.CatzLEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class CatzLED extends VirtualSubsystem {
  public static final CatzLED Instance = new CatzLED();

  private CANdle candle = new CANdle(10);

  // ----------------------------------------------------------------------------------------------
  // Robot state LED tracking
  // ----------------------------------------------------------------------------------------------
  @Getter @Setter @AutoLogOutput (key = "CatzLED/ElevatorLEDState")
  public LEDState curLEDState = LEDState.OFF;
  private LEDState lastLEDState = LEDState.CLIMB;

  public enum LEDState {
    ON,
    OFF,
    STOW,
    DISABLED_BLUE,
    DISABLED_RED,
    CLIMB
  }
  // MISC

  public int loopCycleCount = 0;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kPurple;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;


  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 61;

  private static final int START = 0;
  private static final int END = 52;

  private static final double breathDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double bubbleDuration = 0.25;
  private static final double strobeDuration = 0.25;

  private final SingleFadeAnimation disabledRed;
  private final SingleFadeAnimation disabledBlue;

  private final StrobeAnimation stow;
  private final StrobeAnimation on;
  private final SolidColor off;

  private final RainbowAnimation climb;

  private CatzLED() {
    disabledRed = new SingleFadeAnimation(START, END);
    disabledBlue = new SingleFadeAnimation(START, END);

    stow = new StrobeAnimation(START, END);
    stow.FrameRate = 6.7;
    on = new StrobeAnimation(START, END);
    off = new SolidColor(START, END);

    climb = new RainbowAnimation(START, END);

    disabledRed.Color = new RGBWColor(Color.kRed);
    disabledBlue.Color = new RGBWColor(Color.kBlue);

    stow.Color = new RGBWColor(Color.kRed);
    on.Color = new RGBWColor(Color.kGreen);
    off.Color = new RGBWColor(Color.kRed);

  }

  private void updateControllerState() {
    if (DriverStation.isDisabled()) {
      if(DriverStation.isDSAttached()){
        if (DriverStation.getAlliance().orElseThrow() == Alliance.Blue) {
          curLEDState = LEDState.DISABLED_BLUE;
        }
        else {
          curLEDState = LEDState.DISABLED_RED;
        }
      }else{
        curLEDState = LEDState.CLIMB;
      }
      return;
    }
    if (CatzSuperstructure.Instance.isClimbMode) {
      curLEDState = LEDState.CLIMB;
      return;
    }
    if (!CatzSuperstructure.Instance.isIntakeDeployed) {
      curLEDState = LEDState.STOW;
      return;
    }
    if (CatzSuperstructure.Instance.isIntakeOn) {
      curLEDState = LEDState.ON;
    }
    else {
      curLEDState = LEDState.OFF;
    }
  }

  @Override
  public void periodic() {
    // Update alliance color
    // if (DriverStation.isDSAttached()) {
    //   alliance = DriverStation.getAlliance();
    //   allianceColor =
    //       alliance
    //           .map(alliance -> alliance == Alliance.Blue ? Color.kAqua : Color.kOrangeRed)
    //           .orElse(Color.kPurple);
    //   secondaryDisabledColor = alliance.isPresent() ? Color.kYellow : Color.kBlack;
    // }

    // Update auto state
    // if (DriverStation.isDisabled()) {

    // } else {
    //   lastEnabledAuto = DriverStation.isAutonomous();
    //   lastEnabledTime = Timer.getFPGATimestamp();
    // }

    // Update estop state
    // if (DriverStation.isEStopped()) {
    //   estopped = true;
    // }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    updateControllerState();
    // System.out.println("candle led state "+curLEDState);
    // Update LEDs
    if (curLEDState != lastLEDState) {
      candle.setControl(new EmptyAnimation(0));
      switch (curLEDState) {
        case ON:
          candle.setControl(on);
          break;
        case OFF:
          candle.setControl(off);
          break;
        case STOW:
          candle.setControl(stow);
          break;
        case DISABLED_BLUE:
          candle.setControl(disabledBlue);
          break;
        case DISABLED_RED:
          candle.setControl(disabledRed);
          break;
        case CLIMB:
          candle.setControl(climb);
          break;
      }
    }
    lastLEDState = curLEDState;
  } // end of periodic()


  //-------------------------------------------------------------------------------------------------------
  //
  //    LED factory methods
  //
  //-------------------------------------------------------------------------------------------------------


}
