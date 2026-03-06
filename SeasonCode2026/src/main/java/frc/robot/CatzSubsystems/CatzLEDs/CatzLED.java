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
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
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
  public ShooterLEDState shooterLEDState = ShooterLEDState.nuhthing;
  public IntakeLEDState intakeLEDState = IntakeLEDState.OFF;

  public enum ShooterLEDState {
    FUNCTIONAL,
    AUTO,
    INTAKE_STOW,
    FLYWHEELS,
    endgameAlert,
    nuhthing
  }

  public enum IntakeLEDState {
    ON,
    OFF
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

  private static final int SHOOTER_START = 0;
  private static final int SHOOTER_END = 38;
  private static final int INTAKE_START = 39;
  private static final int INTAKE_END = 54;
  private static final int AURAFARM_START = 55;
  private static final int AURAFARM_END = 60;

  private static final double breathDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double bubbleDuration = 0.25;
  private static final double strobeDuration = 0.25;

  private static final LarsonAnimation functionalRed = new LarsonAnimation(SHOOTER_START, SHOOTER_END);
  private static final LarsonAnimation functionalBlue = new LarsonAnimation(SHOOTER_START, SHOOTER_END);
  private static final LarsonAnimation functionalGreen = new LarsonAnimation(SHOOTER_START, SHOOTER_END);

  private static final LarsonAnimation auto = new LarsonAnimation(SHOOTER_START, SHOOTER_END);

  private static final StrobeAnimation shooting = new StrobeAnimation(SHOOTER_START, SHOOTER_END);
  private static final StrobeAnimation intakeStow = new StrobeAnimation(SHOOTER_START, SHOOTER_END);
  private static final StrobeAnimation endgameAlert = new StrobeAnimation(SHOOTER_START, SHOOTER_END);
  private static final RainbowAnimation nuhthing = new RainbowAnimation(SHOOTER_START, SHOOTER_END);
  private static final SolidColor intakeOn = new SolidColor(INTAKE_START, INTAKE_END);
  private static final SolidColor intakeOff = new SolidColor(INTAKE_START, INTAKE_END);

  private static final RainbowAnimation aurafarm = new RainbowAnimation(AURAFARM_START, AURAFARM_END);

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

    functionalRed.Color = new RGBWColor(Color.kRed);
    functionalBlue.Color = new RGBWColor(Color.kBlue);
    functionalGreen.Color = new RGBWColor(Color.kGreen);

    auto.Color = new RGBWColor(Color.kAqua);

    shooting.Color = new RGBWColor(Color.kGreen);
    intakeStow.Color = new RGBWColor(Color.kRed);
    endgameAlert.Color = new RGBWColor(Color.kPurple);

    intakeOn.Color = new RGBWColor(Color.kGreen);
    intakeOff.Color = new RGBWColor(Color.kRed);

    candle.setControl(aurafarm);
  }

  private void updateControllerState() {
    if (CatzSuperstructure.Instance.isIntakeOn) {
      intakeLEDState = IntakeLEDState.ON;
    }
    else {
      intakeLEDState = IntakeLEDState.OFF;
    }

    if (DriverStation.isJoystickConnected(4)) {
      shooterLEDState = ShooterLEDState.FUNCTIONAL;
      return;
    }
    if (DriverStation.isAutonomous()) {
      if (!CatzSuperstructure.Instance.isIntakeDeployed) {
        shooterLEDState = ShooterLEDState.INTAKE_STOW;
      }
      else {
        shooterLEDState = ShooterLEDState.AUTO;
      }
      return;
    }
    if (DriverStation.isTeleop() && DriverStation.getMatchTime() < 30.0) {
      shooterLEDState = ShooterLEDState.endgameAlert;
      return;
    }
    if (!CatzSuperstructure.Instance.isIntakeDeployed) {
      shooterLEDState = ShooterLEDState.INTAKE_STOW;
      return;
    }
    if (CatzSuperstructure.Instance.getIsScoring()) {
      shooterLEDState = ShooterLEDState.FLYWHEELS;
      return;
    }
    shooterLEDState = ShooterLEDState.nuhthing;
    return;
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

    // Update LEDs
    switch (intakeLEDState) {
      case ON:
        candle.setControl(intakeOn);
      case OFF:
        candle.setControl(intakeOff);
    }
    switch (shooterLEDState) {
      case FUNCTIONAL:
        if (LimelightSubsystem.Instance.isSeeingApriltag()) {
          candle.setControl(functionalGreen);
        }
        else {
          if (alliance.get() == Alliance.Blue) {
            candle.setControl(functionalBlue);
          }
          else {
            candle.setControl(functionalRed);
          }
        }
      case AUTO:
        candle.setControl(auto);
      case INTAKE_STOW:
        candle.setControl(intakeStow);
      case FLYWHEELS:
        candle.setControl(shooting);
      case endgameAlert:
        candle.setControl(endgameAlert);
      case nuhthing:
        candle.setControl(nuhthing);
    }

    ledStrip.setData(buffer);
  } // end of periodic()


  //-------------------------------------------------------------------------------------------------------
  //
  //    LED factory methods
  //
  //-------------------------------------------------------------------------------------------------------

  // LED STROBE
  private void strobe(Color c1, Color c2, double duration) { // duration is the length of one color HUNTER IDEA
    if(Timer.getFPGATimestamp() % duration == 0){
      if(buffer.getLED(0) == c1){
        solid(c2);
      }
      else {
        solid(c1);
      }
    }
  }

  // LED Rainbow
  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < length; i++) {
        x += xDiffPerLed;
        x %= 180.0;
        buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void bubble(Color color, double duration) {
    double numCycles = Timer.getFPGATimestamp() / duration;
    int numFullCycles = (int) Math.floor(numCycles);
    for (int i=0; i<length; i++) {
      if (i % 3 == numFullCycles%3) {
        buffer.setLED(i, color);
      }
      else {
        buffer.setLED(i, Color.kBlack);
      }
    }
  }

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
