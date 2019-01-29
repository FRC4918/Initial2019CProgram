/*---------------------------------------------------------------------------*/
/* Team 4918 Robot                                                           */
/*---------------------------------------------------------------------------*/

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

// all the following are already included by frc/WPILib.h above
// #include <frc/TimedRobot.h>
// #include <frc/Joystick.h>
// #include <frc/Spark.h>
// #include <frc/Compressor.h>
// #include <frc/DoubleSolenoid.h>
// #include <frc/DriverStation.h>
// #include <frc/drive/DifferentialDrive.h>
// #include <frc/AnalogInput.h>

using std::cout;
using std::endl;

/**
 * This sample program shows how to control several motors using a joystick.
 * In the operator control part of the program, the joystick is read and the
 * value is used to determine how to drive the motor.
 *
 * Joystick analog values range from -1 to 1, and speed controller inputs of
 * m_motor*.Set() also range from -1 to 1, so those 2 work together easily;
 * other motor drive methods require some scaling.
 */
class Robot : public frc::TimedRobot {
   private:
      static void VisionThread() {
         // This function executes as a separate thread, to take 640x480 pixel
         // video frames from the USB video camera, change to grayscale, and
         // send to the DriverStation. It is documented here:
         // https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/
         //         669166-using-the-cameraserver-on-the-roborio
         cs::UsbCamera camera =
                     frc::CameraServer::GetInstance()->StartAutomaticCapture();
         // camera.SetResolution(640, 480);
         camera.SetResolution(160, 120);
         cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
         cs::CvSource outputStreamStd =
                  frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
         cv::Mat source;
         cv::Mat output;
         while(true) {
            usleep(1000);
            if ( cvSink.GrabFrame(source) )
            {
               cvtColor(source, output, cv::COLOR_BGR2GRAY);
               outputStreamStd.PutFrame(output);
            }
         }
      }

   public:
      void RobotInit() override {
         std::thread visionThread(VisionThread);
         visionThread.detach();
      }

      void AutonomousInit() override {
         iAutoCount = 0;
         m_motorLSSlave1.Follow(m_motorLSMaster);
         m_motorLSSlave2.Follow(m_motorLSMaster);
         m_motorRSSlave1.Follow(m_motorRSMaster);
         m_motorRSSlave2.Follow(m_motorRSMaster);
#ifndef JAG_NOTDEFINED
         m_compressor.SetClosedLoopControl(true);  // turn compresser on
#else
         m_compressor.SetClosedLoopControl(false); // turn compressor off
#endif
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
         m_motorArmMaster.ConfigNominalOutputForward(0, 10);
         m_motorArmMaster.ConfigNominalOutputReverse(0, 10);
         m_motorArmMaster.ConfigPeakOutputForward(1, 10);
         m_motorArmMaster.ConfigPeakOutputReverse(-1,10);
      }

      void AutonomousPeriodic() override {
                  // This is a simple demonstration of an autonomous sequence.
                  // It uses the iAutoCount variable, which is incremented
                  // each time this function is called (about every 20 ms,
                  // which is about 50 times/sec) to determine what to do.
         if ( iAutoCount < 100 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
         } else if ( iAutoCount < 200 ) {
           m_drive.CurvatureDrive( 0.5, 0.5, 0 );
         } else if ( iAutoCount < 201 ) {
           m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
         } else if ( iAutoCount < 600 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
         } else if ( iAutoCount < 700 ) {
           m_drive.CurvatureDrive( 0.5, 0.5, 0 );
         } else {
           m_drive.CurvatureDrive( 0.0, 0.0, 0 );
         }
         iAutoCount++;
         if ( 750 < iAutoCount ) iAutoCount = 0;
      }

      void TeleopInit() override {
         m_motorLSSlave1.Follow(m_motorLSMaster);
         m_motorLSSlave2.Follow(m_motorLSMaster);
         m_motorRSSlave1.Follow(m_motorRSMaster);
         m_motorRSSlave2.Follow(m_motorRSMaster);
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
         m_motorArmMaster.ConfigNominalOutputForward(0, 10);
         m_motorArmMaster.ConfigNominalOutputReverse(0, 10);
         m_motorArmMaster.ConfigPeakOutputForward(1, 10);
         m_motorArmMaster.ConfigPeakOutputReverse(-1,10);
                   // set ramp speed to 1.0 seconds from Neutral to Full power
         m_motorArmMaster.ConfigClosedloopRamp( 1.0, 10 );

         m_motorArmMaster.ConfigPeakCurrentLimit( 50, 10 );  // 50 Amps
         m_motorArmMaster.ConfigPeakCurrentDuration( 200, 10 );  // 200 msecs
         m_motorArmMaster.ConfigContinuousCurrentLimit( 40, 10 );  // 40 Amps
         m_motorArmMaster.EnableCurrentLimit( true );

         m_motorArmMaster.ConfigSelectedFeedbackSensor(
                                      FeedbackDevice::CTRE_MagEncoder_Relative,
                                      0, 0 );
         m_motorArmMaster.SetSensorPhase( true );
         m_motorArmMaster.Config_kF( 0, 0.40000, 0 );   // was 0.456
         m_motorArmMaster.Config_kP( 0, 0.8, 0 );
         m_motorArmMaster.Config_kI( 0, 0.0, 0 ); // if used, make this
                                                  // very small, like 0.006
         m_motorArmMaster.Config_kD( 0, 1.0, 0 );

         DistSensor1.SetOversampleBits( 4 );
         DistSensor1.SetAverageBits( 2 );
         DistSensor1.SetSampleRate( 3200 );
      }

      void TeleopPeriodic() override {

         static int iCallCount = 0;

         double limex = limenttable->GetNumber("tx",0.0);
         double limey = limenttable->GetNumber("ty",0.0);
         double limev = limenttable->GetNumber("tv",0.0);

         iCallCount++;
         
         m_compressor.ClearAllPCMStickyFaults(); // clear sticky faults in PCM
         // pdp.ClearStickyFaults(); // clear sticky faults in PDP

         // jagorigwas: m_motor.Set(m_stick.GetY());
         // m_motorRSMaster.Set(ControlMode::PercentOutput, m_stick.GetY());
         // m_motorLSMaster.Set(ControlMode::PercentOutput, m_stick.GetY());
         //                   NOTE: check all of these joystick assumptions:
         //    m_stick.GetX -- left (-) or right (+) stick (roll)
         //    m_stick.GetY -- forward (+) or back (-) stick
         //    m_stick.GetZ -- left (-) or right (+) stick (yaw)
         //    m_stick.GetThrottle -- forward (-1.0) or back (+1.0) paddle

         if ( m_stick.GetRawButton(3) ) {      // If button 3 is pressed, then
                                               // drive in simple arcade mode
            m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
         } else if ( m_stick.GetRawButton(5) ) { 
                                              // else if button 5 pressed, then
                                              // drive toward limelight target
cout << "Target Valid: " << limev << ", " << limex << "/" << limey << endl;

            if (limev == 1) {   // if Limelight videocam has acquired a target
               if ( limex > 1 ) {
                    // if target to the right, turn towards the right
                  m_drive.CurvatureDrive( 0.35, -(limex/30.0), 1 );
               } else if (limex < -1 ) {
                 // else if target to the left, turn towards the left
                  m_drive.CurvatureDrive( 0.35, -(limex/30.0), 1 );
               }
            }
         } else { // else neither button 3 or 5 are pressed; do curvature drive
            if ( m_stick.GetZ() < 0 ) {
               m_drive.CurvatureDrive( m_stick.GetY(),
                                       powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
                                       m_stick.GetTop() );
            } else {
               m_drive.CurvatureDrive( m_stick.GetY(),
                                       -powl( fabs(m_stick.GetZ()), m_stick.GetThrottle()+2.0 ),
                                       m_stick.GetTop() );
            }
         }
#ifndef JAG_NOTDEFINED
         m_compressor.SetClosedLoopControl(true); // turn compressor on
#else
         m_compressor.SetClosedLoopControl(false); // turn compressor off
#endif
         // Compressor *c = new Compressor(0);
         // m_doublesolenoid.Set(DoubleSolenoid.Value.kOff);
         if ( m_stick.GetTrigger() ) {
            m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // hi gear
         } else {
            m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // lo gear
         }
         // m_doublesolenoid.Set(DoubleSolenoid::Value::kReverse); // hi gear
         if ( !m_stick.GetRawButton(4) ) { // If button 4 (bottom right, of the
                                           // 4 buttons on top of the joystick)
                                           // is NOT pressed, then move the arm
                                           // with the Talon's PID control.
            /* Speed mode */
            /*
             * 4096 Units/Rev * 328 RPM / 600 100ms/min in either direction:
             * velocity setpoint is in units/100ms
             * We found the max speed of this motor/gearbox combo to be
             * about 2240 ticks/100ms, or about 328 RPM.
             */
            double targetVelocity_UnitsPer100ms = 0.0; 

                       // If button 5 (top left on top of joystick) is pressed,
            if ( m_stick.GetRawButton(5) ) {
                                 // then set speed to exactly -1000 units/100ms
               // cout << "button5 " << endl;
               // exactly 146 RPM to the left (-1000 ticks/100ms)
               targetVelocity_UnitsPer100ms = -1000;
               IntakeMotors.Set(-0.5);   // temporary, for testing trial intake

                 // else if button 6 (top right on top of joystick) is pressed,
            } else if ( m_stick.GetRawButton(6) ) {
                                  // then set speed to exactly 1000 units/100ms
               // cout << "button6 " << endl;
               // exactly 146 RPM to the right (1000 ticks/100ms)
               targetVelocity_UnitsPer100ms = 1000.0;
               IntakeMotors.Set(0.5);    // temporary, for testing trial intake
            } else {   // else neither button 5 or 6 are pressed, so set the
                       // speed according to joystick Z (yaw) under PID control
                     /* up to 328 RPM (2240 ticks/100ms) in either direction */
               targetVelocity_UnitsPer100ms = m_stick.GetZ() * 4096 * 328.0 / 600;
               IntakeMotors.Set(0);      // temporary, for testing trial intake
            }
#ifdef JAG_NOTDEFINED
                        // temporary: test of limelight controlling arm motor
            if (limev == 1){       // if limelight thinks it has a valid target
               if (limex >= 0.0){             // if that target is to the right
                  targetVelocity_UnitsPer100ms = 750;      // turn arm to right
               } else {
                  targetVelocity_UnitsPer100ms = -750;      // turn arm to left
               }
               if (limey >= 0.0){           // if the target is high, move fast
                  targetVelocity_UnitsPer100ms = targetVelocity_UnitsPer100ms * 2;
               } else {                     // else target is low, so move slow
                  targetVelocity_UnitsPer100ms = targetVelocity_UnitsPer100ms * 0.5;
               }
            }
#endif
                            // Display actual/desired speed every 10 iterations
            if ( 0 == (iCallCount%10) )
            {
                          // turn the arm at the speed which was decided above,
                          // using the Talon PID controller in Velocity mode,
               m_motorArmMaster.Set( ControlMode::Velocity,
                                     targetVelocity_UnitsPer100ms);

               //cout << "speed=";
 #ifdef JAG_NOTDEFINED
               cout << cout.fill(' ');
               cout << cout.width(6) << (int)m_motorArmMaster.GetSelectedSensorVelocity(0);
               cout << "/";
               cout << cout.width(6) << (int)targetVelocity_UnitsPer100ms;
               cout << ", (";
               cout << cout.width(6) << (int)targetVelocity_UnitsPer100ms
                                        - (int)m_motorArmMaster.GetSelectedSensorVelocity(0);
               cout << ", ";
               cout << cout.width(6) << m_motorArmMaster.GetMotorOutputPercent();
               cout << "% )";
               cout << "|";
 #endif
            }
         } else {            // else button 4 IS pressed, so move the Arm motor
                             // directly according to joystick Z (yaw) (no PID)
            m_motorArmMaster.Set(ControlMode::PercentOutput, m_stick.GetZ());
         }

         if ( 1 == (iCallCount%50) ) {
            cout << " Dist=";
            cout << DistSensor1.GetValue();
            cout << "/";
            cout << DistSensor1.GetAverageValue();
            cout << " ( ";
            cout << DistSensor1.GetVoltage();
            cout << "/";
            cout << DistSensor1.GetAverageVoltage();
            cout << " )";
            // cout << limenttable->GetNumber("tx",0.0);
            // cout << pdp.GetVoltage() << " |" ;

            // cout << pdp.GetTotalCurrent() << " |" ;
            //cout << pdp.GetTotalPower() << " |" ;
            //cout << pdp.GetTotalEnergy() << " |" ;
            //cout << pdp.GetCurrent(0) << " |"  ;
            //cout << pdp.GetCurrent(1) << " |"  ;
            //cout << pdp.GetCurrent(2) << " |"  ;
            //cout << pdp.GetCurrent(12) << " |"  ;
            //cout << pdp.GetCurrent(13) << " |"  ;
            //cout << pdp.GetCurrent(14) << " |"  ;
            
            cout << endl;

         }

      }

   private:
      frc::Joystick m_stick{0};
      // jagorigwas: frc::Spark m_motor{0};
      WPI_TalonSRX m_motorRSMaster{1}; // Right side drive motor
      WPI_TalonSRX m_motorLSMaster{2}; // Left  side drive motor
      WPI_TalonSRX m_motorArmMaster{7}; // Arm motor
        // Even just instantiating the PowerDistributionPanel here, without 
        // doing anything with the pdp object, causes errors like these
        // to come out continually, about once per second:
        //    ERROR -6 A timeout has been exceeded:
        //               GetTotalCurrent[PowerDistrbutionPanel.cpp:89]
        // Those errors occur with both of the team's Roborios, and both PDPs.
      // frc::PowerDistributionPanel pdp;  
      WPI_VictorSPX m_motorRSSlave1{3};
      WPI_VictorSPX m_motorLSSlave1{4};
      WPI_VictorSPX m_motorLSSlave2{5};
      WPI_VictorSPX m_motorRSSlave2{6};
      int iAutoCount;
      frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
      frc::Compressor m_compressor{0};
      frc::DoubleSolenoid m_doublesolenoid{0,1};
      frc::AnalogInput DistSensor1{0};
      frc::Spark IntakeMotors{0};        // temporary, for testing trial intake
      std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
};

// START_ROBOT_CLASS(Robot)

int main() { return frc::StartRobot<Robot>(); }

