package frc.robot.CatzSubsystems.CatzLEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

public class CatzLED extends VirtualSubsystem {
  public static final CatzLED Instance = new CatzLED();

  // ----------------------------------------------------------------------------------------------
  // Robot state LED tracking
  // ----------------------------------------------------------------------------------------------
  @Getter @Setter @AutoLogOutput (key = "CatzLED/ElevatorLEDState")
  public ControllerLEDState controllerState = ControllerLEDState.nuhthing;

  @Getter @Setter @AutoLogOutput (key = "CatzLED/QueueState")
  public QueueLEDState queueLEDState = QueueLEDState.EMPTY;

  @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/isClimbExtendingOut")
  private WinchingState climbDirection = WinchingState.IDLE;

  public enum QueueLEDState {
    EMPTY,
    ONE_CORAL,
    TWO_CORAL,
    THREE_CORAL,
    FOUR_CORAL
  }

  public enum ControllerLEDState {
    CLIMB,
    FUNCTIONAL,
    AUTO,
    AUTO_INTAKE_STOW,
    INTAKE_STOW,
    FLYWHEELS,
    ROLLERS,
    endgameAlert,
    ledChecked,
    nuhthing
  }

  public enum WinchingState {
    EXTENDING(Color.kGreen),
    RETRACTING(Color.kRed),
    IDLE(Color.kBlack);

    private final Color color;
    private WinchingState(Color color) {
        this.color = color;
    }
}

  public double autoFinishedTime = 0.0;
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
  private static final int length = 54;

  private static final double breathDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double bubbleDuration = 0.25;
  private static final double strobeDuration = 0.25;

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
  }

  private void updateControllerState() {
    if (DriverStation.isJoystickConnected(4)) {
      controllerState = ControllerLEDState.FUNCTIONAL;
      return;
    }
    if (DriverStation.isAutonomous()) {
      if (!CatzSuperstructure.Instance.isIntakeDeployed) {
        controllerState = ControllerLEDState.AUTO_INTAKE_STOW;
      }
      else {
        controllerState = ControllerLEDState.AUTO;
      }
      return;
    }
    if (DriverStation.isTeleop() && DriverStation.getMatchTime() < 30.0) {
      controllerState = ControllerLEDState.endgameAlert;
      return;
    }
    if (CatzSuperstructure.Instance.isClimbMode) {
      controllerState = ControllerLEDState.CLIMB;
      return;
    }
    if (!CatzSuperstructure.Instance.isIntakeDeployed) {
      controllerState = ControllerLEDState.INTAKE_STOW;
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

    // Update LEDs
    switch (controllerState) {
      case CLIMB:
        rainbow(rainbowCycleLength, rainbowDuration);
      case FUNCTIONAL:
        if (LimelightSubsystem.Instance.isSeeingApriltag()) {
          bubble(Color.kGreen, bubbleDuration);
        }
        else {
          if (alliance.get() == Alliance.Blue) {
            bubble(Color.kBlue, bubbleDuration);
          }
          else {
            bubble(Color.kRed, bubbleDuration);
          }
        }
      case AUTO:
        wave(Color.kAqua, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      case AUTO_INTAKE_STOW:
        wave(Color.kRed, Color.kWhite, waveFastCycleLength, waveFastDuration);
      case INTAKE_STOW:
        strobe(Color.kRed, Color.kWhite, strobeDuration);
      case FLYWHEELS:
        strobe(Color.kGreen, Color.kBlack, strobeDuration);
      case ROLLERS:
        strobe(Color.kBlue, Color.kBlack, strobeDuration);
      case endgameAlert:
        strobe(Color.kPurple, Color.kCyan, strobeDuration);
      case ledChecked:
        // TODO
      default:
        solid(Color.kBlack);
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
