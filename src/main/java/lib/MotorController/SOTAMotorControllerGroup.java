// package lib.MotorController;

// import java.util.Arrays;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import lib.Encoder.SOTAEncoder;

// public class SOTAMotorControllerGroup implements SOTAMotorController {
//   private final SOTAMotorController[] mMotorControllers;
//   private boolean kIsInverted;

//   /**
//    * Create a new MotorControllerGroup with the provided MotorControllers.
//    *
//    * @param motorController The first MotorController to add
//    * @param motorControllers The MotorControllers to add
//    */
//   public SOTAMotorControllerGroup(
//     SOTAMotorController motorController, SOTAMotorController... motorControllers) {
//     mMotorControllers = new SOTAMotorController[motorControllers.length + 1];
//     mMotorControllers[0] = motorController;
//   }

//   public SOTAMotorControllerGroup(SOTAMotorController[] motorControllers) {
//     mMotorControllers = Arrays.copyOf(motorControllers, motorControllers.length);
//   }

//   public void set(double speed) {
//     for (SOTAMotorController motorController : mMotorControllers) {
//       motorController.set(kIsInverted ? -speed : speed);
//     }
//   }

//   @Override
//   public double get() {
//     return mMotorControllers[0].get();
//   }

//   @Override
//   public double getTickVelocity() {
//     return mMotorControllers[0].getTickVelocity();
//   }

//   @Override
//   public double getTickPosition() {
//     return mMotorControllers[0].getSensorTickPosition();
//   }

//   // TODO: currently 
//   public SOTAEncoder getEncoder() {
//     return 0;
//   }

//   @Override
//   public double getMotorCurrent() {
//     // TODO Auto-generated method stub
//     return 0;
//   }

//   @Override
//   public double getMotorTemperature() {
//     // TODO Auto-generated method stub
//     return 0;
//   }

// }
