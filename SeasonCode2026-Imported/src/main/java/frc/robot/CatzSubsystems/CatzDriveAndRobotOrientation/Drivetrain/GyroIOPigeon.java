package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.GYRO_ID;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon implements GyroIO {

  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  CANBus bus = new CANBus();

  public GyroIOPigeon() {
    pigeon = new Pigeon2(GYRO_ID, bus);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    // pigeon.getConfigurator().apply(new GyroTrimConfigs().withGyroScalarZ(-3)); //brute force gyro correction code

    yaw.setUpdateFrequency(DriveConstants.GYRO_UPDATE_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.gyroYawVel = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    // if(Math.signum(inputs.gyroYawVel) == 1){
    //   inputs.gyroAngle = yaw.getValueAsDouble() * 0.99;
    // }else if(Math.signum(inputs.gyroYawVel) == -1){
    //   inputs.gyroAngle = yaw.getValueAsDouble() * 1.01;
    // }else{
    //   inputs.gyroAngle = yaw.getValueAsDouble();
    // }
    inputs.gyroAngle = yaw.getValueAsDouble();
  }
}
