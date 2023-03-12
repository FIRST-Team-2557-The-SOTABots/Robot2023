package frc.robot.Util.Configs;

public class SuperStructureConfig {
    private double encoderAtZeroDegrees;
    private double encoderPerDegree;
    private int armBaseLength;
    private double encoderPerInch;
    private double height;
    private double boffset;
    private double foffset;
    private double absoluteoffset;

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
        return boffset;
    }
    public double getfOffset(){
        return foffset;
    }
    public double getAbsoluteOffset(){
        return absoluteoffset;
    }
}

