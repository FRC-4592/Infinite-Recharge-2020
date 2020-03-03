package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;

// Case statement for spinning to precise color
public class ColorWheel implements SubsystemFramework {
    private WPI_VictorSPX colorWheelMotor;
    private ColorSensor colorSensor = new ColorSensor();
    private int spinCounter = 0;
    private String referenceColor = null;

    public ColorWheelPositions state = ColorWheelPositions.Off;

    public ColorWheel(WPI_VictorSPX colorWheelMotor) {
        this.colorWheelMotor = colorWheelMotor;
    }

    public enum ColorWheelPositions {
        Red, Yellow, Green, Blue, Off, Spin;
    }
    
    public boolean GoToRed() {
        return Hardware.driverPad.getRawButton(Constants.RED);
    }
    public boolean GoToBlue() {
        return Hardware.driverPad.getRawButton(Constants.BLUE);
    }
    public boolean GoToGreen() {
        return Hardware.driverPad.getRawButton(Constants.GREEN);
    }
    public boolean GoToYellow() {
        return Hardware.driverPad.getRawButton(Constants.YELLOW);
    }
    public boolean Spin() {
        return Hardware.driverPad.getRawButton(Constants.SPIN);
    }

    public void update() {
        
        ColorWheelPositions newState = state;
        switch(state) {
            case Red:
                colorWheelMotor.set(0.1);
                if (colorSensor.getColorString() == "Red") {// Not exact, I need to account for distance between our wheel and scanner so it might be green here instead of red
                    newState = ColorWheelPositions.Off;
                }
                break;
            case Blue:
                colorWheelMotor.set(1);
                if (colorSensor.getColorString() == "Blue") {
                    newState = ColorWheelPositions.Off;
                }
                break;
            case Green:
                colorWheelMotor.set(1);
                if (colorSensor.getColorString() == "Green") {
                    newState = ColorWheelPositions.Off;
                }
                break;
            case Yellow:
                colorWheelMotor.set(1);
                if (colorSensor.getColorString() == "Yellow") {
                    newState = ColorWheelPositions.Off;
                }
                break;
            case Spin:
                if(referenceColor == null) {
                referenceColor = colorSensor.getColorString();
                }
                else if(colorSensor.getColorString().equals(referenceColor)){
                    spinCounter += 1;
                }
                colorWheelMotor.set(1);
                
                if (spinCounter == 6) {
                    newState = ColorWheelPositions.Off;
                }
                break;
            case Off:
                colorWheelMotor.set(0);
                if (colorSensor.getColorString() != "Unknown") {
                    if(GoToRed()){ 
                        newState = ColorWheelPositions.Red;
                    }
                    else if(GoToBlue()) {
                        newState = ColorWheelPositions.Blue;
                    }
                    else if(GoToGreen()) {
                        newState = ColorWheelPositions.Green;
                    }
                    else if(GoToYellow()) {
                        newState = ColorWheelPositions.Yellow;
                    }
                    else if(Spin()) {
                        newState = ColorWheelPositions.Spin;
                    }
                }
                break;
            default:
                newState = state;
                break;
        }

        if (newState != state) {
            state = newState;
        }
        outputToSmartDashboard();
    }

    public void setupSensors() {
        colorWheelMotor.configNominalOutputForward(0, 10);
        colorWheelMotor.configPeakOutputForward(1, 10);
        colorWheelMotor.configNominalOutputReverse(0, 10);
        colorWheelMotor.configPeakOutputReverse(-1, 10);
        colorSensor.setupSensors();
    }
    
    public void outputToSmartDashboard() {
        
    }
}