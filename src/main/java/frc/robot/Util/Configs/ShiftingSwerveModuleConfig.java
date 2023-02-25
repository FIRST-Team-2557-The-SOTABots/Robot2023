package frc.robot.Util.Configs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShiftingSwerveModuleConfig {
    
    private double[] gearRatios;


    private double angleOffset;
    private double angleEncoderCPR;

    private double wheelCircumference;

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
    private double angleMaxVel;

    private double angleKS;
    private double angleKV;

    private double anglePIDTolerance;
    private double speedPIDTolerance;


    public double[] getGearRatios(){
        return gearRatios;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public double getAngleEncoderCPR() {
        return angleEncoderCPR;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
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
        return angleMaxVel;
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
        new TrapezoidProfile.Constraints(angleMaxVel, angleMaxAccel));
        pid.setTolerance(anglePIDTolerance);
        return pid;
    }
    public ProfiledPIDController speedPID(){
        ProfiledPIDController pid = new ProfiledPIDController(speedKP, speedKI, speedKD,
        new TrapezoidProfile.Constraints(speedMaxVel, speedMaxAccel));
        pid.setTolerance(speedPIDTolerance);
        return pid;
    }

    
}
