/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>

#include <frc/IterativeRobot.h>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/Drive/DifferentialDrive.h>
#include <frc/IterativeRobot.h>
#include <frc/PowerDistributionPanel.h>
#include <ctre/phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <thread>

#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>

#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/Ultrasonic.h>
#include <frc/Relay.h>
#include <frc/Compressor.h>
#include <frc/Servo.h>
#include <frc/Compressor.h>
#include <frc/AnalogGyro.h>
#include <frc/DigitalInput.h>



#undef USE_NAVX
#ifdef USE_NAVX
// NavX-MXP headers
#include "AHRS.h"
#endif

using namespace frc;
using namespace std;
using namespace nt;

class Robot : public frc::IterativeRobot {

	// CAN IDs
    const static int kPdpCanAddress = 0;
    const static int kPcmCanAddress = 1;

    // Channels for the wheels  (CAN IDs)
    const static int frontLeftChannel	= 4;
    const static int rearLeftChannel	= 5;
    const static int frontRightChannel	= 6;
    const static int rearRightChannel	= 7;

    // Joystick port assignments
    const static int kJoystickChannel0	= 0;
    const static int kJoystickChannel1	= 1;

    // Relay port assignments
    const static int kRelayPort = 0;

    // Analog Port assignments
    const static int gyroChannel = 0;

	// Joystick buttons
	const int kRelayForwardButton = 3;     // "X" button on Xbox Controller
	const int kRelayReverseButton = 2;     // "B" button on Xbox Controller

	const int kDoubleSolenoidForward = 4;  // "Y" button on Xbox Controller
	const int kDoubleSolenoidReverse = 1;  // "A" button on Xbox controller

	// Digital port assignments
	const int kLimitSwitchInput = 7;
	const int kUltraSonicPingInput = 8;
	const int kUltraSonicPingOutput = 9;

	// PCM channel assignments
	const int kServoChannel = 9;

	// Miscellaneous constants
	const float Kp = 0.03;
	const double kUpdatePeriod = 0.010;  // update period in seconds

	// Joystick deadband settings
	const float kBottomOfDeadBand = -0.1;
	const float kTopOfDeadBand    =  0.1;

	// Mobility 4WD
    WPI_TalonSRX *m_frontLeft  = new WPI_TalonSRX(frontLeftChannel);
    WPI_TalonSRX *m_rearLeft   = new WPI_TalonSRX(rearLeftChannel);
    WPI_TalonSRX *m_frontRight = new WPI_TalonSRX(frontRightChannel);
    WPI_TalonSRX *m_rearRight  = new WPI_TalonSRX(rearRightChannel);

    SpeedControllerGroup m_left{*m_frontLeft, *m_rearLeft};
    SpeedControllerGroup m_right{*m_frontRight, *m_rearRight};
    DifferentialDrive m_robotDrive{m_left, m_right};

    // Joysticks
	XboxController m_stick{kJoystickChannel0};

	// Pneumatics
	DoubleSolenoid piston{kPcmCanAddress, 0, 7};

	// Relays
	Relay spikeRelay{kRelayPort, Relay::kBothDirections};

	// Object for dealing with the Power Distribution Panel (PDP).
	PowerDistributionPanel m_pdp{kPdpCanAddress};

	// Compressor channel -- not really needed but we'll keep it for completeness
	Compressor  roboCompressor{kPcmCanAddress};

	// Gyro on the roboRio
	AnalogGyro  analogGyro{gyroChannel};

	// Digital IO declarations
	DigitalInput  limitSwitch{kLimitSwitchInput};

	// Ultrasonic Range finder
	Servo      ultrasonicServo{kServoChannel};
	Ultrasonic sr04Ultrasonic{kUltraSonicPingInput, kUltraSonicPingOutput, Ultrasonic::kInches};

#ifdef USE_NAVX
	// Declare the NAVX
    AHRS *ahrs;
#endif

	// Network Tables Crap
	std::shared_ptr<NetworkTable> table{NULL};

	// Miscellaneous Globals
	int   autoLoopCounter;
	float gyroAngle;

public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Pneumatics
		roboCompressor.SetClosedLoopControl(true);
		piston.Set(DoubleSolenoid::kReverse);

		// Set initial angle on servo
		ultrasonicServo.SetAngle(90);

		// Gyro calibration
		analogGyro.Calibrate();
		analogGyro.Reset();

		// enable camera
		CameraServer::GetInstance()->StartAutomaticCapture();

		// Disable the watchdog timers to keep the warning messages down
		m_robotDrive.SetSafetyEnabled(false);

#ifdef USE_NAVX
    	NetworkTableInstance::GetDefault().GetTable("datatable");
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
        	ahrs = new AHRS(SPI::Port::kMXP);
            //ahrs = new AHRS(I2C::Port::kMXP);
            ahrs->EnableLogging(true);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
#endif
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
        autoLoopCounter = 0;
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
			{
				gyroAngle = analogGyro.GetAngle();
				// Do something w/ the Angle
				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed

				autoLoopCounter++;
				} else {
					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
			}
		} else {
			// Default Auto goes here
			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
			{
				gyroAngle = analogGyro.GetAngle();
				// Do something w/ the Angle
				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed

				autoLoopCounter++;
				} else {
					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
			}
		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		int offsetVal = 1;
		int servoAngle = 80;
		int passCount = 0;
		const int numLoopsPerScan = 100;
		float rightY;
		float leftY;
		bool buttonA;
		bool buttonY;
		bool forward;
		bool reverse;

		while (IsOperatorControl()) {

#ifdef USE_NAVX
	        if ( !ahrs ) return;

	        // Uses button 6 on the XBox Controller = RB (right bumper)
	        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,6);
	        if ( reset_yaw_button_pressed ) {
	        	ahrs->
	            ahrs->ZeroYaw();
	        }

	        SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
	        SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
	        SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
	        SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
	        SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
	        SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
	        SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());
	        SmartDashboard::PutNumber(  "IMU_Timestamp",        ahrs->GetLastSensorTimestamp());

	        /* These functions are compatible w/the WPI Gyro Class */
	        SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());
	        SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());

	        SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());
	        SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());
	        SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());
	        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
	        SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());

	        SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
	        SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
	        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
	        SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

	        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
	        /* NOTE:  These values are not normally necessary, but are made available   */
	        /* for advanced users.  Before using this data, please consider whether     */
	        /* the processed data (see above) will suit your needs.                     */

	        SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
	        SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
	        SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
	        SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
	        SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
	        SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
	        SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
	        SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
	        SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
	        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
	        /* Omnimount Yaw Axis Information                                           */
	        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
	        AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
	        SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
	        SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );

	        /* Sensor Board Information                                                 */
	        SmartDashboard::PutString(  "FirmwareVersion",      ahrs->GetFirmwareVersion());

	        /* Quaternion Data                                                          */
	        /* Quaternions are fascinating, and are the most compact representation of  */
	        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
	        /* from the Quaternions.  If interested in motion processing, knowledge of  */
	        /* Quaternions is highly recommended.                                       */
	        SmartDashboard::PutNumber(  "QuaternionW",          ahrs->GetQuaternionW());
	        SmartDashboard::PutNumber(  "QuaternionX",          ahrs->GetQuaternionX());
	        SmartDashboard::PutNumber(  "QuaternionY",          ahrs->GetQuaternionY());
	        SmartDashboard::PutNumber(  "QuaternionZ",          ahrs->GetQuaternionZ());
#endif

			// Implement the deadband
			rightY = -m_stick.GetY(frc::GenericHID::kRightHand);
			leftY  = -m_stick.GetY(frc::GenericHID::kLeftHand);

			if ((rightY > kBottomOfDeadBand) and (rightY < kTopOfDeadBand) )
				rightY = 0;

			if ((leftY > kBottomOfDeadBand) and (leftY < kTopOfDeadBand) )
				leftY = 0;

			// Make it so....
			m_robotDrive.TankDrive(leftY, rightY);

			// Solenoid code
			buttonA = m_stick.GetRawButton(kDoubleSolenoidReverse);
			buttonY = m_stick.GetRawButton(kDoubleSolenoidForward);

			// In order to set the double solenoid, we will say that if neither
			//   button is pressed, it is off, if just one button is pressed,
			//   set the solenoid to correspond to that button, and if both
			//   are pressed, set the solenoid to Forwards.
			if (buttonY)
				piston.Set(DoubleSolenoid::kForward);
			else if (buttonA)
				piston.Set(DoubleSolenoid::kReverse);
			else
				piston.Set(DoubleSolenoid::kOff);

			if (buttonA) {
				// Checks to see if the solenoid has already been set
				// I think that this was a problem last year, so I don't know if this works
				if (piston.Get() != DoubleSolenoid::kReverse)
					piston.Set(DoubleSolenoid::kReverse);
			}

			if (buttonY) {
				// Checks to see if the solenoid has already been set
				// I think that this was a problem last year, so I don't know if this works
				if (piston.Get() != DoubleSolenoid::kForward)
					piston.Set(DoubleSolenoid::kForward);
			}
			// End Solenoid Code

			// Spike Relay Code
			// Retrieve the button values. GetRawButton will return
			//   true if the button is pressed and false if not.
			forward = m_stick.GetRawButton(kRelayForwardButton);
			reverse = m_stick.GetRawButton(kRelayReverseButton);

			// Depending on the button values, we want to use one of
			//   kOn, kOff, kForward, or kReverse.
			// kOn sets both outputs to 12V, kOff sets both to 0V,
			//   kForward sets forward to 12V and reverse to 0V, and
			//   kReverse sets reverse to 12V and forward to 0V.
			if (forward && reverse)
				spikeRelay.Set(Relay::kOn);
			else if (forward)
				spikeRelay.Set(Relay::kForward);
			else if (reverse)
				spikeRelay.Set(Relay::kReverse);
			else
				spikeRelay.Set(Relay::kOff);
			// End Spike Relay Code

			passCount++;

			if ((passCount % numLoopsPerScan) == 0) {
				if (servoAngle <= 80) {
				  offsetVal = 1;
				}
				else if (servoAngle >= 120) {
					offsetVal = -1;
				}

				passCount = 0;
				servoAngle += offsetVal;
				ultrasonicServo.SetAngle(servoAngle);
			}

			if (!(limitSwitch.Get())) {
				// It's closed now
			}

			//Wait(kUpdatePeriod);

		}

	}

	void TestPeriodic() {
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
