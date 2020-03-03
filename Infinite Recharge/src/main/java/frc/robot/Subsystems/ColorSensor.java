package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Lib.SubsystemFramework;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class ColorSensor implements SubsystemFramework {
    private ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();
    private Color detectedColor;
    private String colorString;
    //private double IR;
    //private int proximity;
    private final Color BlueTarget = ColorMatch.makeColor(0.216, 0.444, 0.339);
    private final Color GreenTarget = ColorMatch.makeColor(0.246, 0.494, 0.259);
    private final Color RedTarget = ColorMatch.makeColor(0.361, 0.421, 0.217);
    private final Color YellowTarget = ColorMatch.makeColor(0.305, 0.507, 0.187);
    public ColorSensor() {
        
    }
    public ColorSensor(ColorSensorV3 colorSensor){
        this.colorSensor = colorSensor;
    }
    public void update() {
        // TODO Auto-generated method stub
        this.detectedColor = colorSensor.getColor();
        //this.IR = colorSensor.getIR();
        //this.proximity = colorSensor.getProximity();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        
        if (match.color == BlueTarget){
            colorString = "Blue";
        }
        else if (match.color == RedTarget){
            colorString = "Red";
        }
        else if (match.color == YellowTarget && detectedColor.blue <= 0.200){
            colorString = "Yellow";
        }
        else if(match.color == GreenTarget && detectedColor.blue >= 0.255){
            colorString = "Green";
        }
        else {
            colorString = "Unknown";
        }
        outputToSmartDashboard();
    }
    public String getColorString() {
        return colorString;
    }

    public void setupSensors() {
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);
    }
    
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        SmartDashboard.putString("Color", colorString);
        //SmartDashboard.putNumber("Red", detectedColor.red);
        //SmartDashboard.putNumber("Blue", detectedColor.blue);
        //SmartDashboard.putNumber("Green", detectedColor.green);
        //SmartDashboard.putNumber("IR", IR);
        //SmartDashboard.putNumber("Proximity", proximity);
        //SmartDashboard.putNumber("CGreen", colorSensor.getGreen()/255);
    }
}
