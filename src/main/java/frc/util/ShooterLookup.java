package frc.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Shooter lookup that interpolates between known calibration points.
 * Units:
 *  - distance: meters (or whatever you pick—just be consistent)
 *  - rpm: shooter wheel RPM
 *  - hoodDeg: hood angle in degrees
 */
public class ShooterLookup {

  // Separate interpolators for each output is simplest and robust.
  private final InterpolatingDoubleTreeMap rpmByDistance = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodDegByDistance = new InterpolatingDoubleTreeMap();

  public ShooterLookup() {
    // distance, rpm, hoodDeg
    //Original data
    /*addPoint(1.5, 2400, 53); //layup distance confirmed
    addPoint(2.5, 3100, 51);
    addPoint(2.8000, 3000, 51);
    addPoint(3.3000, 3500, 49);
    addPoint(3.5000, 4500, 46);  /// 3/21 This worked great
    addPoint(4.00, 4500, 45);*/

    addPoint(1.5, 2400, 53); //layup distance confirmed
    addPoint(2.5, 3100, 51);
    addPoint(2.8000, 3000, 51);
    addPoint(3.3000, 3250, 48);   //Configrmed 3/22
    addPoint(4.0, 3750, 48);      //confirmed lower percetage 3/22
    
  }

  /** Adds one calibration point to the table. */
  public void addPoint(double distance, double rpm, double hoodDeg) {
    rpmByDistance.put(distance, rpm);
    hoodDegByDistance.put(distance, hoodDeg);
  }

 

  /** Output setpoints from the lookup. */
  public static record ShooterSetpoint(double rpm, double hoodDeg) {}

  /**
   * Returns interpolated shooter setpoints for the given distance.
   * If distance is outside the table range, WPILib returns the nearest endpoint value.
   */
  public ShooterSetpoint getSetpoint(double distance) {
    double rpm = rpmByDistance.get(distance);
    double hoodDeg = hoodDegByDistance.get(distance);
    return new ShooterSetpoint(rpm, hoodDeg);
  }
}