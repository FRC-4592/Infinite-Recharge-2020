package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;

public class CannonPivot implements SubsystemFramework {
    private static WPI_TalonSRX masterCannonPivot;
    //private static WPI_VictorSPX slaveCannonPivot;
    private Limelight limelight;
    private int degree;

    private static double Average_Ticks_Per_Degree;
    private double CannonPivot_Kp = Constants.CannonPivot_Kp;
    private double CannonPivot_Ki = Constants.CannonPivot_Ki;
    private double CannonPivot_Kd = Constants.CannonPivot_Kd;
    private double CannonPivot_Kf = Constants.CannonPivot_Kf;
    
    public CannonPivot(WPI_TalonSRX masterCannonPivot) {
        this.masterCannonPivot = masterCannonPivot;
        //this.slaveCannonPivot = slaveCannonPivot;

    }
    
    public static CannonPivotStates state = CannonPivotStates.StartPosition;

    public enum CannonPivotStates {
        StartPosition, AngledPosition;
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
        if(StartPosition()) {
            masterCannonPivot.set(0.1);
        }
        else if(AngledPosition()) {
            masterCannonPivot.set(-0.1);
        }
        else {
            masterCannonPivot.set(0);
        }
    }
        /*CannonPivotStates newState = state;
        limelight.Update_Limelight_Tracking();
        switch(state){
            case StartPosition:
                masterCannonPivot.set(ControlMode.Position, setPosition(0));
                degree = 0; // 90 in reality
                if(AngledPosition()) {
                    newState = CannonPivotStates.AngledPosition;
                }
                break;
            case AngledPosition:
                masterCannonPivot.set(ControlMode.Position, setPosition(setupAngle()));
                break;
            default:
                newState = state;
                break;
            }
        if (newState != state) {
            state = newState;
        }
        outputToSmartDashboard();
    } */
    
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Degree", degree);
        SmartDashboard.putNumber("Ticks", masterCannonPivot.getSelectedSensorPosition(0));
    }
    
    public void setupSensors() {
        //slaveCannonPivot.follow(masterCannonPivot);

        masterCannonPivot.setSensorPhase(true);
        
        masterCannonPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0 , 10);
        
        masterCannonPivot.setSelectedSensorPosition(0, 0, 10);
        
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

    public int setupAngle() {
        if (limelight.getLimelightCamtran(2) == 1){
            return 52;
        }
        else if (limelight.getLimelightCamtran(2) == 2){
            return 72;
        }
        else if (limelight.getLimelightCamtran(2) == 3){
            return 85;
        }
        else if (limelight.getLimelightCamtran(2) == 4){
            return 83;
        }
        else if (limelight.getLimelightCamtran(2) == 5){
            return 81;
        }
        else if (limelight.getLimelightCamtran(2) == 6){
            return 51;
        }
        else if (limelight.getLimelightCamtran(2) == 7){
            return 49;
        }
        else if (limelight.getLimelightCamtran(2) == 8){
            return 48;
        }
        else if (limelight.getLimelightCamtran(2) == 9){
            return 47;
        }
        else if (limelight.getLimelightCamtran(2) == 10){
            return 47;
        }
        else if (limelight.getLimelightCamtran(2) == 11){
            return 48;
        }
        else if (limelight.getLimelightCamtran(2) == 12){
            return 49;
        }
        else if (limelight.getLimelightCamtran(2) == 13){
            return 52;
        }
        return 90;
    }
}