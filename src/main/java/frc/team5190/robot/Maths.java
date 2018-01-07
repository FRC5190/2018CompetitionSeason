package frc.team5190.robot;

public class Maths {

    public static float calculateFGain(float power, float feetPerSecond, float wheelSize, float sensorUnitsPerRotation) {
        return (power * 1023) / ((feetPerSecond * 12.0f) / (wheelSize * (float) Math.PI * 2) / 10.0f * sensorUnitsPerRotation);
    }

    public static float calculateFGain(float power, float rpm, float sensorUnitsPerRotation) {
        return (power * 1023) / (rpm / 60f / 10.0f * sensorUnitsPerRotation);
    }

    public static double feetPerSecondToRPM(double feetPerSecond, float wheelRadius) {
        return (feetPerSecond * 12.0f) / (wheelRadius * (float) Math.PI * 2) * 60f;
    }

}
