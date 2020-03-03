package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;

public class CannonPolycord implements SubsystemFramework {
    private WPI_VictorSPX polycordMasterMotor;
    private WPI_VictorSPX polycordSlaveMotor;
    private int time = 0;
    private double Average_Ticks_Per_Inch;
    private double Cannonpolycord_Kf;
    private double Cannonpolycord_Kp;
    private double Cannonpolycord_Ki;
    private double Cannonpolycord_Kd;

    private DigitalInput limiter;
    
    public CannonpolycordStates state = CannonpolycordStates.Off;
    
    public CannonPolycord(WPI_VictorSPX polycordMasterMotor, WPI_VictorSPX polycordSlaveMotor) {
        this.polycordMasterMotor = polycordMasterMotor;
        this.polycordSlaveMotor = polycordSlaveMotor;
    }

    public enum CannonpolycordStates {
        Off, Index, Shooting;
    }
    public boolean Shooting() {
        return(Hardware.driverPad.getRawButton(Constants.SHOOT));
    }
    
    public void update() {
        CannonpolycordStates newState = state;
        switch(state) {
            case Off:
                polycordMasterMotor.set(0);
                if(!limiter.get()){
                    newState = CannonpolycordStates.Index;
                }
                break;
            case Index:
                polycordMasterMotor.set(0.1);
                time += 1;
                if(Shooting()) {
                    newState = CannonpolycordStates.Shooting;
                }
                else if(time == 2) {
                    newState = CannonpolycordStates.Off;
                }
                break;
            case Shooting:
                polycordMasterMotor.set(0.5);
                if(!Shooting()) {
                    newState = CannonpolycordStates.Off;
                }
                break;
            default:
                newState = CannonpolycordStates.Off;
                break;
        }
        if (newState != state) {
            state = newState;
        }
        outputToSmartDashboard();
    }

    public void setupSensors() {
        polycordSlaveMotor.follow(polycordMasterMotor);

        polycordMasterMotor.setSensorPhase(true);
        // Not sure if this is need so will check manually
        polycordSlaveMotor.setInverted(true);

        polycordMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

        polycordMasterMotor.setSelectedSensorPosition(0 , 0, 10);

        polycordMasterMotor.configNominalOutputForward(0, 10);
		polycordMasterMotor.configNominalOutputReverse(0, 10);
		polycordMasterMotor.configPeakOutputForward(-1, 10);
		polycordMasterMotor.configPeakOutputReverse(1, 10);
		
		//Set allowable closed-loop error
		polycordMasterMotor.configAllowableClosedloopError(0, 0, 10);
		
		//Set closed loop gains in PID slot 0
		polycordMasterMotor.config_kF(0, Cannonpolycord_Kf, 10);
		polycordMasterMotor.config_kP(0, Cannonpolycord_Kp, 10);
		polycordMasterMotor.config_kI(0, Cannonpolycord_Ki, 10);
		polycordMasterMotor.config_kD(0, Cannonpolycord_Kd, 10);
    }

    public void outputToSmartDashboard() {
        
    }
}