package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

public class LinearServo extends Servo {
    double m_speed;
    double m_length;
    double setPos;
    double curPos;
    double lastTime = 0;
    
    final double microsecondsPerSecond = 1000000;
    int secondsToMicroseconds(double seconds){
        return (int)(seconds * microsecondsPerSecond);
    }

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
    */
    public LinearServo(int channel, int length, int speed) {
        super(channel);
        setBoundsMicroseconds( 
            secondsToMicroseconds(2.0), 
            secondsToMicroseconds(1.8), 
            secondsToMicroseconds(1.5), 
            secondsToMicroseconds(1.2), 
            secondsToMicroseconds(1.0)
        );
        m_length = length;
        m_speed = speed;
    }
    
    public void setPosition(double setpoint){
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed( (setPos/m_length *2)-1);
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
    */
    public void updateCurPos(){
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed *dt){
            curPos -= m_speed *dt;
        } else if(curPos < setPos - m_speed *dt){
            curPos += m_speed *dt;
        } else {
            curPos = setPos;
        }
    }

    /**
     * Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()} periodically
     *  @return Servo Position [mm]
    */
    public double getPosition(){
        return curPos;
    }

    /**
     * Checks if the servo is at its target position, must be calling {@link #updateCurPos()updateCurPos()} periodically
     * @return true when servo is at its target
    */
    public boolean isFinished(){
        return curPos == setPos;
    }
}