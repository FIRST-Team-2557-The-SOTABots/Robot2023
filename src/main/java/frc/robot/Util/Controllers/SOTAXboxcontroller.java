package frc.robot.Util.Controllers;

import java.util.HashMap;

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

}
