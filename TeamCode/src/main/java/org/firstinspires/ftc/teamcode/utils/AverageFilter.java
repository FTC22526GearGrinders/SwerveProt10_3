package org.firstinspires.ftc.teamcode.utils;

public class AverageFilter {

    private double sum = 0.0;
    private int count = 0;


    public AverageFilter(){}

    public  void addReading(double reading) {
        sum += reading;
        count++;
    }

    public double getAverage() {
        return count > 0 ? sum / count : 0.0;
    }

    public  void reset() {
        sum = 0.0;
        count = 0;
    }

    public  int getCount() {
        return count;
    }
}