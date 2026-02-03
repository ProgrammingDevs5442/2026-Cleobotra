package frc.robot;

import edu.wpi.first.math.MathUtil;

public class Brain {

    public Brain() {

    }

    //#region Hood

    private boolean _hoodActive = false;

    public void setHoodActive(boolean value) {
        this._hoodActive = value;
    }

    public boolean isHoodActive() {
        return this._hoodActive;
    }

    /** Desired position of the hood (in percentage of available range) */
    private double _hoodPosition = 0;

    /** Sets the Desired position of the hood (in percentage of available range) */
    public void setHoodPosition(double value) {
        // Should we ignore this request if the hoos is inactive?
        this._hoodPosition = MathUtil.clamp(value, 0, 1);
    }

    // public void setHoodAngle(double angleInDegrees) {
    //     //TODO Map the angle to position
    //     // Adjust the given angle to be within our limits
    //     double angle = MathUtil.clamp(angleInDegrees, 45, 70);
        
    //     // Adjust that angle to what the position needs to be
    //     // 0 position means what angle
    //     // MAX position means what angle
    //     this.setHoodPosition(.5); // Just set to 50% until we can calculate the true value
    // }

    /** Desired position of the hood (in percentage of available range) */
    public double getHoodPosition() {
        return this._hoodPosition;
    }

    //#endregion (Hood)

}
