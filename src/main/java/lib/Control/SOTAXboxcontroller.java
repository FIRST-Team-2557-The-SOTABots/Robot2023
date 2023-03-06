package lib.Control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SOTAXboxcontroller extends CommandXboxController{

    public SOTAXboxcontroller(int port) {
        super(port);
    }
    public boolean getA(){
        return super.a().getAsBoolean();
    }
    public boolean getB(){
        return super.b().getAsBoolean();
    }
    public boolean getX(){
        return super.x().getAsBoolean();
    }
    public boolean getY(){
        return super.y().getAsBoolean();
    }
    @Override
    public double getLeftX() {
        return Math.abs(super.getLeftX()) < 0.1 ? 0 : super.getLeftX();
    }
    @Override
    public double getLeftY() {
        return Math.abs(super.getLeftY()) < 0.1 ? 0 : super.getLeftY();
    }@Override
    public double getRightX() {
        return Math.abs(super.getRightX()) < 0.1 ? 0 : super.getRightX();
    }@Override
    public double getRightY() {
        return Math.abs(super.getRightY()) < 0.1 ? 0 : super.getRightY();
    }
}
