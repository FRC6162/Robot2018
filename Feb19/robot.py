#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""
import ctre
import wpilib
import wpilib.drive


class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # Pneumatics:
        self.leftGearShift = wpilib.Solenoid(5,0)
        self.rightGearShift = wpilib.Solenoid(5,1)
        self.goldenArrowhead = wpilib.Solenoid(5,2) # Reference to Guyanese flag



        # Include limit switches for the elevator and shoulder mechanisms
        # 2018-2-16 Warning! The Switch's channel should be modified according to the robot! - Fixed
        self.SW0 = wpilib.DigitalInput(0)
        self.SW1 = wpilib.DigitalInput(1)
        self.SW2 = wpilib.DigitalInput(2)
        self.SW3 = wpilib.DigitalInput(3)

        # Left Motor Group Setup
        self.M0 = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.M1 = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        self.M0.setInverted(True)
        self.M1.setInverted(True)
        self.left = wpilib.SpeedControllerGroup(self.M0,self.M1)

        # Right Motor Group Setup
        self.M2 = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.M3 = ctre.wpi_talonsrx.WPI_TalonSRX(1)
        self.right = wpilib.SpeedControllerGroup(self.M2,self.M3)

        # Drive setup
        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)

        # Misc Setting
        self.stick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

        # E = Elevator
        self.E1 = wpilib.VictorSP(0)
        self.E2 = wpilib.VictorSP(1)
        # Shoulder
        self.S1 = wpilib.VictorSP(2)
        self.S2 = wpilib.VictorSP(3)



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
        else:
            self.drive.arcadeDrive(0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        # Drive setting
        self.drive.arcadeDrive(-1*self.stick.getRawAxis(0), self.stick.getRawAxis(1))

        # Elevator
        # 2018-2-16 Warning! The Switch number should be modified accroding to the robot! - Fixed
        if self.stick.getRawButton(1) == True: # & self.SW0.get() == False & self.SW1.get() == False:
            self.E1.set(0.8)
            self.E2.set(-0.8)
        elif self.stick.getRawButton(2) == True: #& self.SW2.get() == False & self.SW3.get() == False:
            self.E1.set(-0.8)
            self.E2.set(0.8)
        else:
            self.E1.set(0)
            self.E2.set(0)

        # Shoulder
        if self.stick.getRawButton(3)==True:
            self.S1.set(0.25)
            self.S2.set(0.25)
        elif self.stick.getRawButton(4)==True:
            self.S1.set(-0.25)
            self.S2.set(-0.25)
        else:
            self.S1.set(0)
            self.S2.set(0)

        #Pneumatics
	#Powercube collector - Golden Arrowhead
        if self.stick.getRawButton(5)==True:
            self.goldenArrowhead.set(True)
        elif self.stick.getRawButton(6)==True:
            self.goldenArrowhead.set(False)
        
	#Shift Gears
        if self.stick.getRawButton(7)==True:
            self.leftGearShift.set(True)
            self.rightGearShift.set(True)
        elif self.stick.getRawButton(8)==True:
            self.leftGearShift.set(False)
            self.rightGearShift.set(False)

if __name__ == "__main__":
    wpilib.run(MyRobot)

