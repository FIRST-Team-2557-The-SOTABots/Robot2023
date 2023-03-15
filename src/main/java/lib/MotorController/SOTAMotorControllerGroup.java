package lib.MotorController;

import java.util.Arrays;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import lib.Encoder.SOTAEncoder;

public class SOTAMotorControllerGroup implements SOTAMotorController {
  private final SOTAMotorController[] mMotorControllers;
  private boolean kIsInverted;

  /**
   * Create a new MotorControllerGroup with the provided MotorControllers.
   *
   * @param motorController The first MotorController to add
   * @param motorControllers The MotorControllers to add
   */
  public SOTAMotorControllerGroup(
    SOTAMotorController motorController, SOTAMotorController... motorControllers) {
    mMotorControllers = new SOTAMotorController[motorControllers.length + 1];
    mMotorControllers[0] = motorController;
  }

  public SOTAMotorControllerGroup(SOTAMotorController[] motorControllers) {
    mMotorControllers = Arrays.copyOf(motorControllers, motorControllers.length);
  }

  public void set(double speed) {
    for (SOTAMotorController motorController : mMotorControllers) {
      motorController.set(kIsInverted ? -speed : speed);
    }
  }

@Override
public double get() {
    return 0; //TODO: finish delegate
}

@Override
public void setInverted(boolean isInverted) {
    kIsInverted = isInverted;
    
}

@Override
public boolean getInverted() {
    return kIsInverted;
}

@Override
public void disable() {
    for(SOTAMotorController motor : mMotorControllers){
        motor.disable();
    }
    
}

@Override
public void stopMotor() {
    for(SOTAMotorController motor : mMotorControllers){
        motor.stopMotor();
    }
}

@Override
public double getTickVelocity() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public SOTAEncoder getEncoder() {
    // TODO Auto-generated method stub
    return null;
}

@Override
public double getMotorCurrent() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public double getMotorTemperature() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public double getPose() {
    // TODO Auto-generated method stub
    return 0;
}


}
