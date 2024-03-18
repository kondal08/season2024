package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FlywheelLookupTable {
    private InterpolatingDoubleTreeMap distanceToRPM, distanceToAngleSetpoint, distanceToShotTime, distanceToFeedTime;
    

    // Distance (meters), rpm, angleSetpoint
    private double[][] lookupTable;

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

    public FlywheelLookupTable(double[][] lookupTable) {
        distanceToRPM = new InterpolatingDoubleTreeMap();
        distanceToAngleSetpoint = new InterpolatingDoubleTreeMap();
        distanceToShotTime = new InterpolatingDoubleTreeMap();
        distanceToFeedTime = new InterpolatingDoubleTreeMap();
        this.lookupTable = lookupTable;

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
