package frc.robot.Util.Configs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShiftingSwerveModuleConfig {

    private int encoderPort;

    private String modulePosition;
    
    private double[] gearRatios;

    private double angleOffset;
    private double angleEncoderCPR;

    private double wheelDiameter;   

    private double speedKP;
    private double speedKI;
    private double speedKD;
    private double speedMaxAccel;
    private double speedMaxVel;

    private double speedKS;
    private double speedKV;

    private double angleKP;
    private double angleKI;
    private double angleKD;
    private double angleMaxAccel;

    private double angleKS;
    private double angleKV;

    private double anglePIDTolerance;
    private double speedPIDTolerance;

    public int getEncoderPort(){
        return encoderPort;
    }

    public double[] getGearRatios(){
        return gearRatios;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public double getAngleEncoderCPR() {
        return angleEncoderCPR;
    }

    /**
     * This needs to exist so that wheelDiameter is assigned a value with objectMapper
     * for now this is not used and would prefer to use getWheelCircumference
     */
    public double getwheelDiameter() {
        return wheelDiameter;
    }

    public double getWheelCircumference() {
        return wheelDiameter * Math.PI;
    }

    public double getSpeedKP() {
        return speedKP;
    }

    public double getSpeedKI() {
        return speedKI;
    }

    public double getSpeedKD() {
        return speedKD;
    }

    public double getSpeedMaxAccel() {
        return speedMaxAccel;
    }

    public double getSpeedMaxVel() {
        return speedMaxVel;
    }

    public double getSpeedKS() {
        return speedKS;
    }

    public double getSpeedKV() {
        return speedKV;
    }

    public double getAngleKP() {
        return angleKP;
    }

    public double getAngleKI() {
        return angleKI;
    }

    public double getAngleKD() {
        return angleKD;
    }

    public double getAngleMaxAccel() {
        return angleMaxAccel;
    }

    public double getAngleMaxVel() {
        return angleMaxAccel * Math.sqrt((angleEncoderCPR / 4) / angleMaxAccel);
    }

    public double getAngleKS() {
        return angleKS;
    }

    public double getAngleKV() {
        return angleKV;
    }

    public double getAnglePIDTolerance() {
        return anglePIDTolerance;
    }
    private double getSpeedPIDTolerance() {
        return speedPIDTolerance;
    }

    public SimpleMotorFeedforward angleFF(){
        return new SimpleMotorFeedforward(angleKS, angleKV);
    }
    public SimpleMotorFeedforward speedFF(){
        return new SimpleMotorFeedforward(speedKS, speedKV);
    }
    public ProfiledPIDController anglePID(){
        ProfiledPIDController pid = new ProfiledPIDController(angleKP, angleKI, angleKD, 
        new TrapezoidProfile.Constraints(getAngleMaxVel(), angleMaxAccel));
        pid.setTolerance(anglePIDTolerance);
        pid.enableContinuousInput(0, angleEncoderCPR);
        return pid;
    }
    public ProfiledPIDController speedPID(){
        ProfiledPIDController pid = new ProfiledPIDController(speedKP, speedKI, speedKD,
        new TrapezoidProfile.Constraints(speedMaxVel, speedMaxAccel));
        pid.setTolerance(speedPIDTolerance);
        return pid;
    }

    
}
