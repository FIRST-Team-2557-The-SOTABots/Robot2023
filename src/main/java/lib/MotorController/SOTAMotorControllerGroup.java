package lib.MotorController;

import java.util.Arrays;

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

@Override
public double getNativeEncoderPose() {
    // TODO Auto-generated method stub
    return 0;
}
@Override
    public MotorLimits getMotorLimits() {
        return null;
    }

@Override
public void setVoltage(double voltage) {
    // TODO Auto-generated method stub
    
}

@Override
public double getTickPosition() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public double getNativeVelocity() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public double getNativePosition() {
    // TODO Auto-generated method stub
    return 0;
}

@Override
public SOTAEncoder getNativeEncoder() {
    // TODO Auto-generated method stub
    return null;
}

}
