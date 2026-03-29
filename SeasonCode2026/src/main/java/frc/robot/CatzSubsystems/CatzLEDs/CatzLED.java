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

  // LED IO
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // LED PWM IDs
  private final int LEADER_LED_PWM_PORT = 0;

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
    ledStrip = new AddressableLED(LEADER_LED_PWM_PORT);
    buffer = new AddressableLEDBuffer(length); // NOTE -WPILIB doesn't support creation of 2 led objects
    ledStrip.setLength(length);
    ledStrip.setData(buffer);
    ledStrip.start();

    loadingNotifier = new Notifier(
                            () -> {
                              synchronized (this) {
                                breath(Color.kBlack, Color.kWhiteSmoke, System.currentTimeMillis() / 1000.0);
                                ledStrip.setData(buffer);
                              }
                            }
    );
    loadingNotifier.startPeriodic(0.02);

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
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        curLEDState = LEDState.DISABLED_BLUE;
      }
      else {
        curLEDState = LEDState.DISABLED_RED;
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
    if (DriverStation.isDSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kAqua : Color.kOrangeRed)
              .orElse(Color.kPurple);
      secondaryDisabledColor = alliance.isPresent() ? Color.kYellow : Color.kBlack;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {

    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

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
    ledStrip.setData(buffer);
  } // end of periodic()


  //-------------------------------------------------------------------------------------------------------
  //
  //    LED factory methods
  //
  //-------------------------------------------------------------------------------------------------------

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
    for (int i = (int) Math.ceil(MathUtil.clamp(length * percent, 0, length)); i<length; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  private void solid(Color color) {
    solid(1, color);
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }
}
