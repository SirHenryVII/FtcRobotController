package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ScalerMap {
    private List<Double> percents = new ArrayList<>();
    private List<Double> scalers = new ArrayList<>();

    public void add(double percent, double scaler) {
        percents.add(percent);
        scalers.add(scaler);
    }

    public Double get(Double status) {
        double minY = 0;
        double minX = 0;
        double maxY = 0;
        double maxX = 0;
        for(int i = 1; i < percents.size(); i++) {
            minY = scalers.get(i-1);
            minX = percents.get(i-1);
            maxY = scalers.get(i);
            maxX = percents.get(i);
            if (status < percents.get(i)) break;
        }

        //mx+b!
        double m = (maxY - minY)/(maxX - minX);
        double b = minY - m * minX;
        return m * status + b;
    }
}
