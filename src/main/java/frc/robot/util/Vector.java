package frc.robot.util;

public class Vector {
    private double x;
    private double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getMagnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    public double getAngle() {
        return Math.toDegrees(Math.atan2(y, x));
    }

    public static Vector add(Vector vector1, Vector vector2) {
        double newVectorX = vector1.getX() + vector2.getX();
        double newVectorY = vector1.getY() + vector2.getY();
        return new Vector(newVectorX, newVectorY);
    }

    public static Vector multiply(Vector vector, double multiplier) {
        double newVectorX = vector.getX() * multiplier;
        double newVectorY = vector.getY() * multiplier;
        return new Vector(newVectorX, newVectorY);
    }
}
