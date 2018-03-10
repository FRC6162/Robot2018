#!/usr/bin/en5v python3
"""
    This is a good foundation to build your robot code on
"""
import ctre
import wpilib
import wpilib.drive
from networktables import NetworkTables
#from robotpy_ext.autonomous import AutonomousModeSelector



class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        #Initialize Networktables
        self.sd = NetworkTables.getTable('SmartDashboard')

        #Camera:
        wpilib.CameraServer.launch()
        #Counters
        self.getCubeCounter = 0
        self.dropCubeCounter = 0
        self.elevatorDownCounter = 0
        self.elevatorUpCounter = 0
        self.cubeTravelUp = 50
        self.cubeTravelStop = 1
        #Flags
        self.prepareCubeFlag=0
        self.grabCubeFlag=0
        self.deliverCubeFlag=0

        #Drive Factor - adjust controller responsiveness
        self.driveFactor = 0.5

        #Encoders - left and right, attached to gearbox
        self.EC1 = wpilib.Encoder(4,5)
        self.EC2 = wpilib.Encoder(6,7)

        # Pneumatics:
        self.leftGearShift = wpilib.Solenoid(5,0)
        self.rightGearShift = wpilib.Solenoid(5,1)
        self.goldenArrowhead = wpilib.Solenoid(5,2) # Reference to Guyanese flag

        # Include limit switches for the elevator and shoulder mechanisms
        # 2018-2-16 Warning! The Switch's channel should be modified according to the robot! - Fixed
        self.SW0 = wpilib.DigitalInput(0) #Lower Elevator Switch
        self.SW1 = wpilib.DigitalInput(1) #Upper Elevator Switch
        #self.SW2 = wpilib.DigitalInput(2) #Lower shoulder switch
        #self.SW3 = wpilib.DigitalInput(3) #Upper shoulder switch

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
        self.drive.setMaxOutput(self.driveFactor)

        # Misc Setting
        self.stick = wpilib.Joystick(0)
        self.timer = wpilib.Timer()

        # E = Elevator
        self.E1 = wpilib.VictorSP(0)
        self.E2 = wpilib.VictorSP(1)
        # Shoulder
        self.S1 = wpilib.VictorSP(2)
        self.S2 = wpilib.VictorSP(3)
        
        #Servo
        self.SV1 = wpilib.Servo(4)
        #self.SV2 = wpilib.Servo(5)
        #self.SV1.set(0.0)
        #self.SV2.set(0.0)
        
        #Gyro
        self.gyro = wpilib.ADXRS450_Gyro(0)
        self.gyro.reset()
        
        #Encoder for the shoulder
        self.EC3 = wpilib.Encoder(6,7)
        self.EC4 = wpilib.Encoder(8,9)
        #All possible autonomous routines in a sendable chooser
        self.chooser = wpilib.SendableChooser()
        self.chooser.addDefault("None", '4')
        self.chooser.addObject("left-LeftScale", '1')
        self.chooser.addObject("Middle-LeftScale", '2')
        self.chooser.addObject("Right-LeftScale", '3')
        self.chooser.addObject("Left-RightScale", '5')
        wpilib.SmartDashboard.putData('Choice', self.chooser)
    



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #All possible autonomous routines in a sendable chooser
        #Each possibility has a list of items:
        #Example Start position 1 + scale: straight 3.5m turn 90 right forward 1.0m deliver cube
        self.timer.reset()
        self.timer.start()
        self.autoState = 0
        #self.auto = self.chooser.getSelected()
        '''
        self.cumulativeTime=0
        self.totalTime=0
        self.dataSet=[[-0.5,0,1,-1.0],[0.3,0.4,1,1.0],[-0.5,0,1,-1.0]]
        for i in self.dataSet:
            self.totalTime+=i[2]
        self.intervals = 0
        self.currentTime = 0
        for i in range(0,len(self.dataSet)):
            self.dataSet[i].append([self.currentTime,self.currentTime+self.dataSet[i][2]])
            self.currentTime+=self.dataSet[i][2]
        for i in self.dataSet:
            if i[3]==1.0:
                i.append("Forward")
            if i[3]==-1.0:
                i.append("Backward")
                
        self.timer.reset()
        self.timer.start()
        '''
        #self.EC1.reset()
        #self.EC2.reset()
        #self.EC1.setDistancePerPulse(0.01)        
        
            
    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        '''
        for i in self.dataSet:
            if i[4][0] < self.timer.get() and self.timer.get() <= i[4][1]:
                self.drive.arcadeDrive(i[0],i[1])
                self.SV1.set(i[3])
                self.sd.putValue("Camera",i[5])
            else:
                self.drive.arcadeDrive(0,0)
        '''       
        # Drive for two seconds
        '''
        if self.leftEncoder.getDistance() <= 1.0:
            self.drive.arcadeDrive(-0.5, 0)
        else:
            self.drive.arcadeDrive(0,0)
        '''
        '''
        Place the robot forward
        #Right -> Right Switch
        if self.auto == 1:
            if self.EC1.getDistance() <= 366: #cm
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() <= 90:
                self.drive.arcadeDrive(0.5,0.4)
            else:
                if self.EC4.getDistance() <= 831:
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Left -> Left Switch
        if self.auto == 2:
            if self.EC1.getDistance() <= 366:
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() >= -90 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,-0.4)
            else:
                if self.EC4.getDistance() <= 831:
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Middle -> Left Switch  
        if self.auto == 3:
            if self.gyro.getAngle() >= -37 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,-0.4)
            elif self.EC1.getDistance() <= 456: #cm
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() >= -180 and self.gyro.getAngle() <= -37:
                self.drive.arcadeDrive(0.5,-0.4)
            else: 
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Middle -> Right Switch
        if self.auto == 4:
            if self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 14:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 375: #cm
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() >= -180 and self.gyro.getAngle() <= -14:
                self.drive.arcadeDrive(0.5,-0.4)
            else: 
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Left -> Right Switch
        if self.auto == 5:
            if self.gyro.getAngle() >=0 and self.gyro.getAngle() <= 60:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 
        '''
        #Gyro: clockwise - positive; counterclockwise - negative
        #Place the robot backwards
        #Right -> Right Switch
        
        if self.auto == 1:
            if self.autoState == 0:
                if self.gyro.getAngle() <= 14:
                    self.drive.arcadeDrive(0.5,-0.4)
                self.autoState = 1
                self.EC1.reset()
            if self.autoState == 1:
                if self.EC1.getDistance() <= 377: #cm
                    self.drive.arcadeDrive(0.6,0)
                self.autoState = 2
            if self.autoState == 2:
                if self.EC4.getDistance() <= 831:
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                self.autoState = 3
            if self.autoState == 3:
                if self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Left -> Left Switch
        if self.auto == 2:
            if self.autoState == 0:
                if self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 14:
                    self.drive.arcadeDrive(0.5,0.4)
                    self.autoState = 1
            if self.autoState == 1:
                if self.EC1.getDistance() <= 377: #cm
                    self.drive.arcadeDrive(0.6,0)
                self.autoState = 2
            if self.autoState == 2:
                if self.EC4.getDistance() <= 831:
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                    self.autoState = 3
            if self.autoState == 3:
                if self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Middle -> Left Switch  
        if self.auto == 3:
            if self.autoState == 0:
                if self.gyro.getAngle() >= -37 and self.gyro.getAngle() <= 0:
                    self.drive.arcadeDrive(0.5,-0.4)
                self.autoState = 1
            elif self.EC1.getDistance() <= 456: #cm
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() >= -37 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,0.4)
            else: 
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Middle -> Right Switch
        if self.auto == 4:
            if self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 14:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 375: #cm
                self.drive.arcadeDrive(-0.6,0)
            elif self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 14:
                self.drive.arcadeDrive(0.5,-0.4)
            else: 
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Left -> Right Switch
        if self.auto == 5:
            if self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 60:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 282: #cm, not accurate
                self.drive.arcadeDrive(0.6,0)
            elif self.gyro.getAngle() >=0 and self.gyro.getAngle() <= 60:
                self.drive.arcadeDrive(0.5,-0.4)
            elif self.EC1.getDistance() <= 402 and self.EC1.getDistance() >= 282:
                self.drive.arcadeDrive(0,6,0)
            else:
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Right -> Left Switch
        if self.auto == 6:
            if self.gyro.getAngle() >= -41 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,-0.4)
            elif self.EC1.getDistance() <= 282:
                self.drive.arcadeDrive(0.6,0)
            elif self.gyro.getAngle() >= -41 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 402 and self.EC1.getDistance() >= 282:
                self.drive.arcadeDrive(0,6,0)
            else:
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
        #Left -> Left Scale
        if self.auto == 7:
            if self.gyro.getAngle() >= -5 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,-0.4)
            elif self.EC1.getDistance() <= 400: #cm
                self.drive.arcadeDrive(0.6,0)
            elif self.gyro.getAngle() >= -5 and self.gyro.getAngle() <= 0:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 762: #cm
                self.drive.arcadeDrive(0.6,0)
            else:
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
         #Right -> Right Scale
        if self.auto == 8:
            if self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 5:
                self.drive.arcadeDrive(0.5,0.4)
            elif self.EC1.getDistance() <= 400: #cm
                self.drive.arcadeDrive(0.6,0)
            elif self.gyro.getAngle() >= 0 and self.gyro.getAngle() <= 5:
                self.drive.arcadeDrive(0.5,-0.4)
            elif self.EC1.getDistance() <= 762: #cm
                self.drive.arcadeDrive(0.6,0)
            else:
                if self.EC4.getDistance() <= 831: #shoulder
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
                elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
                    self.goldenArrowhead.set(False)
                    self.S1.set(-0.25)
                    self.S2.set(-0.25)
          #Left -> Right Scale  
          
          if self.auto == 9:
              
            
            
                
                
            
            
            
            
            
        
        
        
        
        
        
        #Right -> Left Scale
        
            
            
        #place your code here
        #self.EC1.getRate() - Get the current rate of the encoder. Units are distance per second as scaled by the value from setDistancePerPulse().
        
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
	
        #Set the maximum output of the drive based on the left trigger:
        self.drive.setMaxOutput(1.0-self.stick.getRawAxis(3))
        # Drive setting - use left stick for forward drive and right stick for backward drive
        #if self.stick.getRawAxis(4)==0 and self.stick.getRawAxis(5)==0:
        self.drive.arcadeDrive(-1*self.stick.getRawAxis(0), self.stick.getRawAxis(1))
        #if self.stick.getRawAxis(0)==0 and self.stick.getRawAxis(1)==0:
        #    self.drive.arcadeDrive(-1*self.stick.getRawAxis(4),self.stick.getRawAxis(5))

        # Elevator
        # 2018-2-16 Warning! The Switch number should be modified accroding to the robot! - Fixed
        '''
        if self.stick.getRawButton(1) == True: # & self.SW0.get() == False & self.SW1.get() == False:
            self.E1.set(1)
            self.E2.set(-1)
        elif self.stick.getRawButton(2) == True: #& self.SW2.get() == False & self.SW3.get() == False:
            self.E1.set(-1)
            self.E2.set(1)
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
        '''

        #Pneumatics
	#Powercube collector - "Golden Arrowhead"
        if self.stick.getRawButton(1) == True:
            self.prepareCubeFlag = 1
            self.EC3.set(0)
        if self.prepareCubeFlag > 0:
            self.prepareGrabCube()
        if self.stick.getRawButton(2) == True:
            self.grabCubeFlag = 1
            self.EC3.set(0)
        if self.grabCubeFlag > 0:
            self.grabCube()
        if self.stick.getRawButton(3) == True:
            self.deliverCubeFlag = 1
            self.EC4.set(0)
        if self.deliverCubeFlag > 0:   
            self.deliverCube()
            
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
            
        #Camera Point Front:
        if self.stick.getPOV()==0:
            self.SV1.set(1.0)
            self.camera='Forwards'
        #Camera Point Back:
        if self.stick.getPOV()==180:
            self.SV1.set(-1.0)
            self.camera='Backwards'
        #Adjust left elevators
        if self.stick.getPOV()==90:
            self.E1.set(-0.3)
        
        #Adjust right elevators
        if self.stick.getPOV()==270:
            self.E2.set(0.3)
        
        #State Machine
        if self.stick.getRawButton(5) == True:
            self.goldenArrowhead.set(True)
        if self.stick.getRawButton(6) == True:
            self.goldenArrowhead.set(False)
        '''
        if self.gyro.getAngle() == 0:
            self.E.set(0.5)
            self.sd.putNumber('Gyro',0)
        if self.gyro.getAngle() == 90:
            self.E.set(0)
            self.sd.putNumber('Gyro',90)
        '''
        
        #Dashboard
        self.sd.putNumber('Speed', 0.5)
        self.sd.putNumber('Gyro',self.gyro.getAngle())
        #self.sd.putValue("Camera", self.camera)
        self.sd.putValue("SW1", self.SW1.get())
        self.sd.putValue("SW0", self.SW0.get())
    def prepareGrabCube(self):
    #(1)Check that the lower elevator switch is on - elevator at bottom
	#(2)If not, move elevator to bottom (and arms to bottom)
        if self.EC1.getDistance() <= 30720:
            self.E1.set(0.5)
            self.E2.set(-0.5)
        else:
            self.E1.set(0)
            self.E2.set(0)
            self.prepareCubeFlag = 0
            
    def grabCube(self):
    #(1)Grab cube
    #(2) Move cube up until it hits the top (or part way up????)
        self.goldenArrowhead.set(True) #Grabs the cube(not sure it is True or False)
        if self.EC3.getDistance() >= -30720 and self.EC3.getDistance() <= 0:
            self.E1.set(-0.5)
            self.E2.set(0.5)
        else:
            self.E1.set(0)
            self.E2.set(0)
            self.grabCubeFlag = 0
    
    def deliverCube(self):
        if self.EC4.getDistance() <= 831:
            self.S1.set(-0.25)
            self.S2.set(-0.25)
        elif self.EC4.getDistance() >= 831 and self.EC4.getDistance() <= 887:
            self.goldenArrowhead.set(False)
            self.S1.set(-0.25)
            self.S2.set(-0.25)
        else:
            self.deliverCubeFlag=0
            
    
if __name__ == "__main__":
    wpilib.run(MyRobot)

