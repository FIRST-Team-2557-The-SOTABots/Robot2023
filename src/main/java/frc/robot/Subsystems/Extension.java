package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;
import lib.MotorController.SOTA_MotorController;

public class Extension extends SubsystemBase{
    private SOTA_MotorController mMotor;
    private DigitalInput mLimitswitch;
    private double kMaxLength;
    private double kArmBaseLength;
    private double kEncoderPerInch;
    private boolean hasReset = false;

    public Extension(SOTA_MotorController motor, DigitalInput limitSwitch, SuperStructureConfig config){
        this.mMotor = motor; 
        this.mLimitswitch = limitSwitch; 
        this.kMaxLength = config.getMaxExtension();
        this.kArmBaseLength = config.getArmBaseLength();
        this.kEncoderPerInch = config.getEncoderPerInch();
    }

    public void setVoltage(double speed){
        SmartDashboard.putNumber("SpeedInit", speed);
        if(mLimitswitch.get() && speed < 0){
           speed = 0;
        }
        if(!hasReset && speed > 0) speed = 0;
        mMotor.setVoltage(speed);
        SmartDashboard.putNumber("extensionSpeed", speed);
    }
 

    public double getEncoder(){
        return mMotor.getEncoderPosition();
    }

    public double getLength(){
        return kArmBaseLength + getLengthFromStart();
    }   

    public double getLengthFromStart() {
        return Math.max((getEncoder() / kEncoderPerInch), 0 );
    }

    public boolean isFullyRetracted(){
        return mLimitswitch.get();
    }
    public double getMaxExtension(){
        return kMaxLength;
    }

    @Override
    public void periodic() {
        if(mLimitswitch.get()) {
            mMotor.resetNativeEncoder();
            hasReset = true;
        }
        SmartDashboard.putNumber("extensionEncoder", getEncoder());
        SmartDashboard.putNumber("length", getLengthFromStart());
        // SmartDashboard.putNumber("Extension length inches", getLength());
        SmartDashboard.putBoolean("limitswitch", mLimitswitch.get());
        // SmartDashboard.putNumber("extension motor limit", motor.getMotorLimits().getUpperLimit());

    }

}
