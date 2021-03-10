package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.networktables.*;

public class CannonPivot implements SubsystemFramework {
    private static WPI_TalonSRX masterCannonPivot;
    // private static WPI_VictorSPX slaveCannonPivot;
    public Limelight limelight = new Limelight();
    private int degree;
    private int angle;

    private static double Average_Ticks_Per_Degree = Constants.Average_Ticks_Per_Degree;
    private double CannonPivot_Kp = Constants.CannonPivot_Kp;
    private double CannonPivot_Ki = Constants.CannonPivot_Ki;
    private double CannonPivot_Kd = Constants.CannonPivot_Kd;
    private double CannonPivot_Kf = Constants.CannonPivot_Kf;

    public CannonPivot(WPI_TalonSRX masterCannonPivot) {
        this.masterCannonPivot = masterCannonPivot;
        // this.slaveCannonPivot = slaveCannonPivot;

    }
    public CannonPivot() {

    }
    public static CannonPivotStates state = CannonPivotStates.StartPosition;

    public enum CannonPivotStates {
        StartPosition, AngledPosition, ShootPosition;
    }

    public double setPosition(double pos) {
        return pos * Average_Ticks_Per_Degree;
    }

    public static boolean testSafePosition(double Safe_Position) {
        return (Safe_Position * Average_Ticks_Per_Degree) < masterCannonPivot.getSelectedSensorPosition(0);
	}

	public static boolean testSafeHighPosition (double Safe_High_Position) {
		return (Safe_High_Position * Average_Ticks_Per_Degree) >= masterCannonPivot.getSelectedSensorPosition(0);
    }
    
    public static boolean StartPosition() {
        return(Hardware.driverPad.getRawButton(Constants.STARTPOSITION));
    }

    public static boolean AngledPosition() {
        return(Hardware.driverPad.getRawButton(Constants.ANGLEDPOSITION));
    }

    public void update() { 
       /* if(StartPosition()) {
            masterCannonPivot.set(0.45);
        }
        else if(AngledPosition()) {
            masterCannonPivot.set(-0.45);
        }
        else {
            masterCannonPivot.set(0);
        }
        outputToSmartDashboard();
    } */
        CannonPivotStates newState = state;
        limelight.Update_Limelight_Tracking(); //23 - 67 32 - 
        angle = (int)findAngle();
        SmartDashboard.putNumber("angle", angle);
        switch(state){
            case StartPosition:
                masterCannonPivot.set(ControlMode.Position, setPosition(-23));
                degree = 80; //Degree we start at 
                SmartDashboard.putString("Pivot", "Start");
                if(AngledPosition()) {
                    newState = CannonPivotStates.AngledPosition;
                }
                else if(StartPosition()) {
                    newState = CannonPivotStates.ShootPosition;
                }
                break;
            case AngledPosition:
                //double adjustedDegree = degree - findAngle();
                //masterCannonPivot.set(ControlMode.Position, setPosition(adjustedDegree));
                SmartDashboard.putString("Pivot", "Angled");
                if(StartPosition()) {
                    newState = CannonPivotStates.StartPosition;
                }
                break;
            case ShootPosition:
                double adjustedDegree = degree - angle;
                SmartDashboard.putNumber("Adjusteddegree", adjustedDegree);
                if (adjustedDegree > -23 && adjustedDegree < 30) {
                masterCannonPivot.set(ControlMode.Position, setPosition(adjustedDegree));
                }
                SmartDashboard.putString("Pivot", "ShootPosition");
                if(AngledPosition()) {
                    newState = CannonPivotStates.AngledPosition;
                }
                else if(StartPosition()) {
                    newState = CannonPivotStates.StartPosition;
                }
                else {
                    newState = CannonPivotStates.ShootPosition;
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
    
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Degree", degree);
        SmartDashboard.putNumber("Ticks", masterCannonPivot.getSelectedSensorPosition());
    }
    
    public void setupSensors() {
        //slaveCannonPivot.follow(masterCannonPivot); //65 Angle start position 

        masterCannonPivot.setSensorPhase(true);
        
        masterCannonPivot.setSelectedSensorPosition(0, 0, 10);
        
        masterCannonPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        masterCannonPivot.configNominalOutputForward(0, 10);
        masterCannonPivot.configNominalOutputReverse(0, 10);
        masterCannonPivot.configPeakOutputForward(1, 10);
        masterCannonPivot.configPeakOutputReverse(-1, 10);

        masterCannonPivot.configAllowableClosedloopError(0, 0, 10);

        masterCannonPivot.config_kF(0, CannonPivot_Kf, 10); 
	    masterCannonPivot.config_kP(0, CannonPivot_Kp, 10);
	    masterCannonPivot.config_kI(0, CannonPivot_Ki, 10);
        masterCannonPivot.config_kD(0, CannonPivot_Kd, 10);
        
    }
    public double findAngle() {
        //SmartDashboard.putNumber("Angle", limelight.getLimelightDistance());
        if(Math.atan(Math.toRadians(limelight.getLimelightDistance()/98.25)) >= 45) {
            return Math.atan(Math.toRadians(limelight.getLimelightDistance()/98.25));
        }
        return 45; //Adjust distance a little for cannon purposes
    }
}