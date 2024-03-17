package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FlywheelLookupTable {
    private static FlywheelLookupTable instance;

    public static FlywheelLookupTable getInstance() {
        if (instance == null)
            instance = new FlywheelLookupTable();
        return instance;
    }

    private InterpolatingDoubleTreeMap distanceToRPM, distanceToAngleSetpoint, distanceToShotTime, distanceToFeedTime;
    

    // Distance (meters), rpm, angleSetpoint
    private double[][] lookupTable = {
            { 1, 2600,-115-10, 0.0, 0.0},
            { 1.45, 2600, -87-10, 0.0, 0.0},
            { 1.95, 2600, -67, 0.0, 0.0},
            { 2.47, 3000, -52+5, 0.0, 0.0},
            //  { 2.49, 3700, -52- 12},
            //  { 2.90, 3800, -52- 12},
            //  { 2.96, 4000, -30 - 12},
            { 3.48, 4000, -19 +5, 0.0, 0.0},
            { 3.95, 4050, -10 +5, 0.0, 0.0},
            { 4.43, 4200, -4, 0.0, 0.0}
    };

/*
 *  { 0.815, 2400,-115},
            { 1.315, 2400, -87},
            { 1.815, 2500, -67},
            { 2.315, 2700, -52},
            { 2.815, 4000, -30},
            { 3.315, 4000, -19},
            { 3.815, 4050, -10},
            { 4.315, 4200, -4}
    
 */

    private FlywheelLookupTable() {
        distanceToRPM = new InterpolatingDoubleTreeMap();
        distanceToAngleSetpoint = new InterpolatingDoubleTreeMap();
        distanceToShotTime = new InterpolatingDoubleTreeMap();
        distanceToFeedTime = new InterpolatingDoubleTreeMap();

        createShootMap(lookupTable);
    }

    private void createShootMap(double[][] table) {
        for (double[] t : table) {
            Double d = (t[0]);
            distanceToRPM.put(d, (t[1] + 1000) );
            distanceToAngleSetpoint.put(d, t[2]* 15/25);
            distanceToShotTime.put(d, t[3]);
            distanceToFeedTime.put(d, t[4]);
        }
    }

    public ActionSetpoint get(Double d) {
        ActionSetpoint values = new ActionSetpoint(
                distanceToRPM.get(d),
                distanceToAngleSetpoint.get(d),
                distanceToShotTime.get(d),
                distanceToFeedTime.get(d));
        return values;
    }

    public void clearTables() {
        distanceToAngleSetpoint.clear();
        distanceToRPM.clear();
        distanceToShotTime.clear();
        distanceToFeedTime.clear();
    }

}
