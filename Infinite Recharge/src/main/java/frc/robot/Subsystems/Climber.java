package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Util.doubleSolenoid;

public class Climber implements SubsystemFramework {
    private WPI_TalonSRX wenchMasterMotor;
    private WPI_VictorSPX wenchSlaveMotor;
    private doubleSolenoid shifter3;
    
    public ClimberStates state = ClimberStates.Off;
    
    public Climber(WPI_TalonSRX wenchMasterMotor, WPI_VictorSPX wenchSlaveMotor, doubleSolenoid shifter3) {
        this.wenchMasterMotor = wenchMasterMotor;
        this.wenchSlaveMotor = wenchSlaveMotor;
        this.shifter3 = shifter3;
    }
    public enum ClimberStates {
        ClimberOpen, WenchPull, Off;
    }
    public boolean ClimberOpen() {
        return Hardware.driverPad.getRawButton(Constants.CLIMBEROPEN);
    }
    public boolean WenchPull() {
        return Hardware.driverPad.getRawButton(Constants.WENCHPULL);
    }



    public void update() {
        ClimberStates newState = state;

        switch(state) {
            case ClimberOpen:
                shifter3.open();
                if(WenchPull()) {
                    newState = ClimberStates.WenchPull;
                }
                break;
            case WenchPull:
                if(WenchPull()) {
                    wenchMasterMotor.set(0.1);
                }
                //wenchMasterMotor.set(ControlMode.Position, 2); // Found through testing
                break;
            case Off:
                wenchMasterMotor.set(ControlMode.Position, 0);
                if(ClimberOpen()) {
                    newState = ClimberStates.ClimberOpen;
                }
                break;
            default:
                newState = ClimberStates.Off;
                break;
        }
        
        if(newState != state){
            state = newState;
        }

        outputToSmartDashboard();
    }
    public void setupSensors() {
        wenchSlaveMotor.follow(wenchMasterMotor);

        wenchMasterMotor.setSensorPhase(true);
        
        wenchMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0 , 10);
        
        wenchMasterMotor.setSelectedSensorPosition(0, 0, 10);
        
        wenchMasterMotor.configNominalOutputForward(0, 10);
        wenchMasterMotor.configNominalOutputReverse(0, 10);
        wenchMasterMotor.configPeakOutputForward(1, 10);
        wenchMasterMotor.configPeakOutputReverse(-1, 10);

        wenchMasterMotor.configAllowableClosedloopError(0, 0, 10);

        wenchMasterMotor.config_kP(0, 0.1, 10); // P value is 0.1
    }
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Ticks", wenchMasterMotor.getSelectedSensorPosition(0));
    }

}