package lib.MotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.Config.MotorControllerConfig;
import lib.Encoder.FalconIntegratedEncoder;
import lib.Encoder.SOTAEncoder;

public class Falcon implements SOTAMotorController {
    private final TalonFX mMotor;
    private final SOTAEncoder mEncoder;
    private final SOTAEncoder mNativeEncoder;

    public Falcon(TalonFX motor, MotorControllerConfig config){
        this(
            motor, 
            new FalconIntegratedEncoder(new TalonFXSensorCollection(motor)), 
            config
        );
        setInverted(config.getInverted());
    }

    public Falcon(TalonFX motor, SOTAEncoder encoder, MotorControllerConfig config) {
        this.mMotor = motor;
        this.mEncoder = encoder;
        this.mNativeEncoder = new FalconIntegratedEncoder(mMotor.getSensorCollection());
    }

    public void set(double speed) {
        mMotor.set(ControlMode.PercentOutput, speed);     
    }

    public void setVoltage(double voltage) {
        set(voltage / RobotController.getBatteryVoltage());
    }

    public double get() {
        return mMotor.getMotorOutputPercent();
    }

    public void setInverted(boolean isInverted) {
        mMotor.setInverted(isInverted);        
    }

    public boolean getInverted() {
        return mMotor.getInverted();
    }

    public void disable() {
        mMotor.neutralOutput();        
    }

    public double getTickVelocity() {
        return mEncoder.getVelocity();
    }

    public double getTickPosition() {
        return mEncoder.getPosition();
    }

    public SOTAEncoder getEncoder() {
        return mEncoder;
    }

    public double getNativeTickVelocity() {
        return mMotor.getSelectedSensorVelocity();
    }

    public double getNativeTickPosition() {
        return mNativeEncoder.getPosition();
    }

    public SOTAEncoder getNativeEncoder() {
        return mNativeEncoder;
    }

    public double getMotorCurrent() {
        return mMotor.getSupplyCurrent();
    }

    public double getMotorTemperature() {
        return mMotor.getTemperature();
    }

}
