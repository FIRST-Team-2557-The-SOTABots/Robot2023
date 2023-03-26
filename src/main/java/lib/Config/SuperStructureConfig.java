package lib.Config;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SuperStructureConfig {
    private double encoderAtZeroDegrees;
    private double encoderPerDegree;
    private int armBaseLength;
    private double encoderPerInch;
    private double height;
    private double bOffset;
    private double fOffset;
    private double fAbsoluteOffset;
    private double bAbsoluteOffset;
    private double maxExtension;
    private double roatationEncoderOffset;
    private double rotationDelta;
    private double rotationDeltaProportional;

    private double rotationInitSetpoint;

    private double rotationKPG;
    private double rotationKP;
    private double rotationKI;
    private double rotationKD;
    private double rotationMaxVel;
    private double rotationMaxAccel;

    private double extensionKP;
    private double extensionKI;
    private double extensionKD;
    private double extensionMaxVel;
    private double extensionMaxAccel;

    public double getRotationDelta() {
        return rotationDelta;
    }

    public double getrotationDeltaProportional() {
        return rotationDeltaProportional;
    }
    public double getEncoderAtZeroDegrees(){
        return encoderAtZeroDegrees;
    }
    public double getEncoderPerDegree(){
        return encoderPerDegree;
    }
    public int getArmBaseLength(){
        return armBaseLength;
    }
    public double getEncoderPerInch(){
        return encoderPerInch;
    }
    public double getHeight(){
        return height;
    }
    public double getbOffset(){
        return bOffset;
    }
    public double getfOffset(){
        return fOffset;
    }
    public double getfAbsoluteOffset(){
        return fAbsoluteOffset;
    }
    public double getbAbsoluteOffset(){
        return bAbsoluteOffset;
    }
    public double getMaxExtension(){
        return maxExtension;
    }

    public double getRoatationEncoderOffset(){
        return roatationEncoderOffset;
    }

    public double getRotationInitSetpoint() {
        return rotationInitSetpoint;
    }

    public double getRotationKPG() {
        return rotationKPG;
    }
    
    public double getRotationKP() {
        return rotationKP;
    }

    public double getRotationKI() {
        return rotationKI;
    }

    public double getRotationKD() {
        return rotationKD;
    }

    public double getRotationMaxVel() {
        return rotationMaxVel;
    }

    public double getRotationMaxAccel() {
        return rotationMaxAccel;
    }

    public double getExtensionKP() {
        return extensionKP;
    }

    public double getExtensionKI() {
        return extensionKI;
    }

    public double getExtensionKD() {
        return extensionKD;
    }

    public double getExtensionMaxVel() {
        return extensionMaxVel;
    }

    public double getExtensionMaxAccel() {
        return extensionMaxAccel;
    }

    public PIDController getRotationPIDController() {
        PIDController pid = new PIDController(rotationKP, rotationKI, rotationKD);
        return pid;
    }

    public ProfiledPIDController getRotationProfiledPIDController() {
        ProfiledPIDController pid = new ProfiledPIDController(rotationKP, rotationKI, rotationKD, 
            new Constraints(rotationMaxVel, rotationMaxAccel));
        return pid;
    }

    public PIDController getExtensionPIDController() {
        PIDController pid = new PIDController(extensionKP, extensionKI, extensionKD);
        return pid;
    }

    public ProfiledPIDController getExtensionProfiledPIDController() {
        ProfiledPIDController pid = new ProfiledPIDController(extensionKP, extensionKI, extensionKD, 
            new Constraints(extensionMaxVel, extensionMaxAccel));
        return pid;
    }


}

