package frc.team5190.robot.navigation;

import static frc.team5190.robot.util.Maths.ftm;

public class Arcade
{

    public enum Position
    {
        PLAYER_STATION_ONE, _PLAYER_STATION_TWO, PLAYER_STATION_THREE;
    }

    static class Pos
    {
        double getX()
        {
            return x;
        }

        double getXMeters()
        {
            return ftm(this.getX());
        }

        public void setX(double x)
        {
            this.x = x;
        }

        double getY()
        {
            return y;
        }

        double getYMeters()
        {
            return ftm(this.getY());
        }

        public void setY(double y)
        {
            this.y = y;
        }

        double getAngle()
        {
            return angle;
        }

        public void setAngle(double angle)
        {
            this.angle = angle;
        }

        private double x;
        private double y;
        private double angle;

        public Pos(double x, double y, double angle)
        {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }

    }
}
