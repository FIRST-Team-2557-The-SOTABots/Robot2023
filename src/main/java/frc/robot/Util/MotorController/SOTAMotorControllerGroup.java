package frc.robot.util.MotorController;

import java.util.Arrays;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SOTAMotorControllerGroup implements SOTAMotorController, Sendable, AutoCloseable {
    private boolean m_isInverted;
  private final SOTAMotorController[] m_motorControllers;
  private static int instances;

  /**
   * Create a new MotorControllerGroup with the provided MotorControllers.
   *
   * @param motorController The first MotorController to add
   * @param motorControllers The MotorControllers to add
   */
  public SOTAMotorControllerGroup(
    SOTAMotorController motorController, SOTAMotorController... motorControllers) {
    m_motorControllers = new SOTAMotorController[motorControllers.length + 1];
    m_motorControllers[0] = motorController;
    System.arraycopy(motorControllers, 0, m_motorControllers, 1, motorControllers.length);
    init();
  }

  public SOTAMotorControllerGroup(SOTAMotorController[] motorControllers) {
    m_motorControllers = Arrays.copyOf(motorControllers, motorControllers.length);
    init();
  }

  private void init() {
    for (SOTAMotorController controller : m_motorControllers) {
      SendableRegistry.addChild(this, controller);
    }
    instances++;
    SendableRegistry.addLW(this, "MotorControllerGroup", instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  @Override
  public void set(double speed) {
    for (SOTAMotorController motorController : m_motorControllers) {
      motorController.set(m_isInverted ? -speed : speed);
    }
  }

  @Override
  public void setVoltage(double outputVolts) {
    for (SOTAMotorController motorController : m_motorControllers) {
      motorController.setVoltage(m_isInverted ? -outputVolts : outputVolts);
    }
  }

  @Override
  public double get() {
    if (m_motorControllers.length > 0) {
      return m_motorControllers[0].get() * (m_isInverted ? -1 : 1);
    }
    return 0.0;
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_isInverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return m_isInverted;
  }

  @Override
  public void disable() {
    for (SOTAMotorController motorController : m_motorControllers) {
      motorController.disable();
    }
  }

  @Override
  public void stopMotor() {
    for (SOTAMotorController motorController : m_motorControllers) {
      motorController.stopMotor();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }

@Override
public double getSensorTickVelocity() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return tick velocity");
}

@Override
public double getTickPosition() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return tick position");

}

@Override
public double getEncoder() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return encoder");

}


@Override
public double getMotorCurrent() {
    int current = 0;
    for (SOTAMotorController controller : m_motorControllers ){
        current += controller.getMotorCurrent();
    }
    return current;
}

@Override
public double getEncoderCountsPerRevolution() throws RuntimeException {
        throw new RuntimeException("MotorControllerGroup Cannot return getEncoderCountsPerRevolution");
    
    
}

@Override
public double getMotorTemperature() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return getMotorTemperature");

}

@Override
public void setLimits(MotorLimits motorLimits) throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot setLimits");

}

@Override
public double getLowerLimit() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return getLowerLimit");

}

@Override
public double getUpperLimit() throws RuntimeException {
    throw new RuntimeException("MotorControllerGroup Cannot return getUpperLimit");

}
}
