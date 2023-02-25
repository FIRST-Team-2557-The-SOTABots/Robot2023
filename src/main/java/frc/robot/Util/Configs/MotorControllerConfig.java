package frc.robot.Util.Configs;

public class MotorControllerConfig  {
    private int port ;
    private boolean isInverted ;
    private String motorType ;
    private int countsPerRevolution ;
    private String idleMode ;

    public boolean getInverted() {
        return isInverted;
    }

    public String getIdleMode() {
        return idleMode;
    }

    public String getMotorType() {
        return motorType;
    }

    public int getCountsPerRevolution() {
        return countsPerRevolution;
    }

    public int getPort(){
        return this.port;
    }
}
