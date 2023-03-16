package lib.Pneumatics;

public interface GearShifter {
    int getGear();
    void shift(int gear);
    int shiftUp();
    int shiftDown();
    double getCurrentGearRatio();
    double getRatioFromGear(int gear);
}
