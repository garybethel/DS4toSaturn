#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch



#define dead_zone 10  //Dead area of analog stick
#define analog_digital_dead_zone 15
uint8_t mode_Selector = 0;  //0 is digital mode 1 is analog

unsigned char DATA1_1, DATA1_2;
unsigned char DATA2_1, DATA2_2;
unsigned char DATA3_1, DATA3_2;
unsigned char DATA4_1, DATA4_2;
unsigned char DATA5_1, DATA5_2;
unsigned char DATA6_1, DATA6_2;
unsigned char DATAEND_1, DATAEND_2;

bool ACK_LINE = 0;
bool power_on = false;


void setup() {


 DATA1_1 = B00001111;                 // DATA1_1
 DATA1_2 = B00001111;                 // DATA1_2
 DATA2_1 = B00001111;                 // DATA2_1
 DATA2_2 = B00001111;                 // DATA2_2
 DATA3_1 = B00001111;                 // DATA3_1
 DATA3_2 = B00001111;                 // DATA3_2 
 DATA4_1 = B00001111;                 // DATA4_1
 DATA4_2 = B00001111;                 // DATA4_2 
 DATA5_1 = B00001111;                 // DATA5_1
 DATA5_2 = B00001111;                 // DATA5_2
 DATA6_1 = B00001111;                 // DATA6_2
 DATA6_2 = B00001111;                 // DATA6_2
 DATAEND_1 = B00000000;
 DATAEND_2 = B00000001;

 DDRC |= B00011111;

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
 Serial.print(F("\r\nPS3 USB Library Started")); 
 
 
 if(managePowerState()){
 //pull the ack line high on startup
 changeACKState();
 }
 //we need to wait for the saturn to startup, 1.5 seconds
 delayMicroseconds(500);
 }

 void loop() {
	
	Usb.Task();
	
	power_on = managePowerState(); //handles powering on and off the saturn
	
	//only if power is on do we want to run this routine
	if(power_on){
		if (PS3.PS3Connected) {
			
			//Changes the controller from Digital to Analog
			if (PS3.getButtonClick(OPTIONS)){
				if(mode_Selector == 0){
					mode_Selector = 1; //change state to now operating in analog mode
					Serial.print(F("\r\nController switched to Analog"));
				}
				else{
					mode_Selector = 0; //change state to now operating in digital mode
					Serial.print(F("\r\nController switched to Digital"));
				}
			}
			
			if(mode_Selector==0){
				emulateDigitalController(); 
				setLed(); //led would change to blue
			}
			else if(mode_Selector==1) {
				emulateAnalogController();
				setLed(); //led would change to green
			}
		}
		else{
			sendDataStateHigh(); //if no PS controller is attached we send the data lines to high emulating no controller for the Saturn
		}
	}
	else{
		sendDataStateLow();//we sent all the data out pin low as to not damage the saturn when its not powered on
	}
 
 }

//this entire function takes 378 us once controller s0 goes low
void emulateDigitalController(){
	
////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0() == 0){
		delayMicroseconds(40);
		//waiting for the saturn
		while(readS1() == 1 && readS0() == 0){
			//between 4 and 24 us wait. Not work messing with
		}
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC =  B00000000;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		
		while(readS1() == 0 && readS0() == 0){
		//12 to 16us wait
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B00000010;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1() == 1 && readS0() == 0){
			//between 28 and 32us never seems to drop below 20
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1() == 0 && readS0() == 0){
			//12 to 16 drops to 8 sometimes
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		//// START SECOND BYTE
		
		//Wait for Saturn to request first half of Data2
		while(readS1() == 1 && readS0() == 0){
			//4 to 8. not to be messed with
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1() == 0 && readS0() == 0){
			//12 to 16 dropping to 8 
		}
		PORTC = DATA2_2;
		changeACKState();
		//// END SECOND BYTE
		
		//// START THIRD BYTE, END OF DATA
		
		// Wait for Saturn to request first half End
		while(readS1() == 1 && readS0() == 0){
			//steady 40 to 44 us never seems to drop below 40
		}
		PORTC = DATAEND_1;
		changeACKState();
		// Wait for Saturn to request second half of End
		while(readS1() == 0 && readS0() == 0){
			//usual 12 to 16 dropping to 8
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		/// END THIRD BYTE
		// Wait for the Saturn to deselect controller	
		//This wait is 104us and was removed so that we begin to poll the Dual Shock controller input earlier. 
		//while(readS0() == 0){}
		DATA1_1 =  B00001111;
		DATA1_2 =  B00001111;
		DATA2_1 =  B00001111;
		DATA2_2 =  B00001111;
		
		uint8_t analogReading;
		//polling the Dual Shock takes  about 76us       this time is out of spec now since adding the twin stick instructions
			
		//If the Up button pressed
		if(PS3.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(PS3.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(PS3.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(PS3.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		/*CIRCLE*/if(PS3.getButtonPress(CIRCLE)){DATA1_2 &= B00001110;}/*CIRCLE*/
		/*R3*/if(PS3.getButtonPress(R3)){/**/}/*R3*/
		/*CROSS*/if(PS3.getButtonPress(CROSS)){DATA1_2 &= B00001011;}/*CROSS*/
		
		//If the Start button pressed
		if (PS3.getButtonPress(PS)){DATA1_2 &= B00000111;}
		
		/*L3*/if(PS3.getButtonPress(L3)){/**/}/*L3*/
		/*TRIANGLE*/if(PS3.getButtonPress(TRIANGLE)){DATA2_1 &= B00001101;}/*TRIANGLE*/
		/*SQUARE*/if(PS3.getButtonPress(SQUARE)){DATA2_1 &= B00001011;}/*SQUARE*/
		/*R1*/if(PS3.getButtonPress(R1)){DATA1_2 &= B00001101;}/*R1*/
        /*L1*/if(PS3.getButtonPress(L1)){DATA2_1 &= B00001110;}/*L1*/
		
		if(PS3.getAnalogButton(R2) >0) {DATA2_1 &= B00000111;}
        if(PS3.getAnalogButton(L2) >0) {DATA2_2 &= B00000111;}
		

		////////stick function assignment 
		analogReading = PS3.getAnalogHat(LeftHatX);
		if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
			//Right
			if(analogReading >= 127 + analog_digital_dead_zone){DATA1_1 &= B00000111;}
			//Left
			if(analogReading <= 127 - analog_digital_dead_zone){DATA1_1 &= B00001011;}
		}
		analogReading = PS3.getAnalogHat(LeftHatY);
		if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
			//Up
			if(analogReading <= 127 -analog_digital_dead_zone){DATA1_1 &= B00001110;}
			//Down
			if(analogReading >= 127 + analog_digital_dead_zone){DATA1_1 &= B00001101;}
		}
		///////////////////////////////////
		
		delayMicroseconds(18);//added since we need to be at 104us wait average..........out of spec but minused 10us from the original time 
	
		while(readS0() == 0){}
		
	}
	
}
//this entire function takes 650 us once controller s0 goes low
void emulateAnalogController(){

	////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0() == 0){
		delayMicroseconds(40);
		
		//waiting for the saturn
		while(readS1() == 1 && readS0() == 0){
		}
		
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC = B00000001;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		while(readS1() == 0 && readS0() == 0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B0000110;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1() == 1 && readS0() == 0){
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1() == 0 && readS0() == 0){
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		///// START SECOND BYTE
		//Wait for Saturn to request first half of Data2
		while(readS1() == 1 && readS0() == 0){
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1() == 0 && readS0() == 0){
		}
		PORTC = DATA2_2;
		changeACKState();
		
		///// END SECOND BYTE
		
		///// START THIRD BYTE,
		
		// Wait for Saturn to request first half of Data3
		while(readS0() == 0 && readS1() == 1){
		}
		PORTC = DATA3_1;
		changeACKState();
		// Wait for Saturn to request second half of Data3
		while(readS0() == 0 && readS1() == 0){
		}
		PORTC = DATA3_2;
		changeACKState();

		//// END THIRD BYTE
		
		///// START FOURTH BYTE,
		
		// Wait for Saturn to request first half of Data4
		while(readS0() == 0 && readS1() == 1){
		}
		PORTC = DATA4_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data4
		while(readS0() == 0 && readS1() == 0){
		}
		PORTC = DATA4_2;
		changeACKState();
		///// END FOURTH BYTE
		
		///// START FIFTH BYTE,
		
		// Wait for Saturn to request first half of Data5
		while(readS0() == 0 && readS1() == 1){
		} 
		PORTC = DATA5_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data5
		while(readS0() == 0 && readS1() == 0){
		}
		PORTC = DATA5_2;
		changeACKState();
		
		///// END FIFTH BYTE

		///// START SIXTH BYTE,
		
		// Wait for Saturn to request first half of Data6
		while(readS0() == 0 && readS1() == 1){
		}
		PORTC = DATA6_1;
		changeACKState();
		// Wait for Saturn to request second half of Data6
		while(readS0() == 0 && readS1() == 0){
		}
		PORTC = DATA6_2;
		changeACKState();
		///// END SIXTH BYTE
		
		///// START SEVENTH BYTE, END OF DATA
		// Wait for Saturn to request first half End
		while(readS0() == 0 && readS1() == 1){
		}
		PORTC = DATAEND_1;
		changeACKState();
		
		// Wait for Saturn to request second half of End
		while(readS0() == 0 && readS1() == 0){
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		///// END SEVENTH BYTE
		
		// Wait for the Saturn to deselect controller
		/*while(readS0() == 0){
		}*/
		
		///// END COMMS WITH SATURN
		
		DATA1_1 = B00001111;
		DATA1_2 = B00001111;
		DATA2_1 = B00001111;
		DATA2_2 = B00001111;
		///////////////////
		DATA3_1 = B00000111;
		DATA3_2 = B00001111;
		DATA4_1 = B00000111;
		DATA4_2 = B00001111;
		DATA5_1 = B00001111;
		DATA5_2 = B00001111;
		DATA6_1 = B00001111;
		DATA6_2 = B00001111;
		
		uint8_t analogReading;
		
		//If the Up button pressed
		if(PS3.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(PS3.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(PS3.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(PS3.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		/*CIRCLE*/if(PS3.getButtonPress(CIRCLE)){DATA1_2 &= B00001110;}/*CIRCLE*/
		/*R3*/if(PS3.getButtonPress(R3)){/**/}/*R3*/
		/*CROSS*/if(PS3.getButtonPress(CROSS)){DATA1_2 &= B00001011;}/*CROSS*/
		
		//If the Start button pressed
		if (PS3.getButtonPress(PS)){DATA1_2 &= B00000111;}
		
		/*L3*/if(PS3.getButtonPress(L3)){/**/}/*L3*/
		/*TRIANGLE*/if(PS3.getButtonPress(TRIANGLE)){DATA2_1 &= B00001101;}/*TRIANGLE*/
		/*SQUARE*/if(PS3.getButtonPress(SQUARE)){DATA2_1 &= B00001011;}/*SQUARE*/

		/*R1*/if(PS3.getButtonPress(R1)){DATA2_1 &= B00000111;}/*R1*/
		/*L1*/if(PS3.getButtonPress(L1)){DATA2_2 &= B00000111;}/*L1*/
		
		//Data3
		analogReading = PS3.getAnalogHat(LeftHatX);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,DATA3_1,DATA3_2);
		}
		//Data4
		analogReading = PS3.getAnalogHat(LeftHatY);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,DATA4_1,DATA4_2);
		}
		//Data5
		analogReading = PS3.getAnalogButton(L2);
		if(analogReading >0) {
			analogReading = (255-analogReading);
			getBinary(analogReading,DATA5_1,DATA5_2);
			//in the event the controller uses digital triggers in analog mode
			DATA2_2 &= B00000111;
		}
		//Data6
		analogReading = PS3.getAnalogButton(R2);
		if(analogReading >0) {
			analogReading = (255-analogReading);
			getBinary(analogReading,DATA6_1,DATA6_2);
			//in the event the controller uses digital triggers in analog mode
			DATA2_1 &= B00000111;
		}
	}	
}


//use to determine the status of the ACK return line and send back the opposite.eg. If its low now then return a high;
 void changeACKState(){
  if(ACK_LINE == 0){
	PORTC |= B00010000;
	ACK_LINE = 1;
  }
  else{
	PORTC &=~ B00010000;
	ACK_LINE = 0;
  }
 }


 void sendDataStateLow(){
  PORTC &= ~B00001111;
 }
 
 void sendDataStateHigh(){
  PORTC |= B00001111;
 }

 void getBinary(uint8_t num, unsigned char &first_nibble, unsigned char &second_nibble){
	unsigned char temp;
	temp = char(num);
	second_nibble = (second_nibble & 0b11110000) | (temp & 0b00001111);
	temp = temp >> 4;
	first_nibble = (first_nibble & 0b11110000) | (temp & 0b00001111);
 }
 
bool readS0(){
 if(PIND & (1 << PD3)){
	return 1;
 }
 else{
	return 0;
 }
}

bool readS1(){
 if(PIND & (1 << PD4)){
	return 1;
 }
 else{
	return 0;
 }
}

//checks if the saturn is powered on by reading the 5v line that would have been used on the standard controller for power
bool readPowerState(){
 if(PIND & (1 << PD5)){
	return 1;
 }
 else{
	return 0;
 }
}

 bool managePowerState(){

	if(readPowerState() == 1){
		return true;
	}
	else{
		return false;
	}
 }


void setLed(){

    if(mode_Selector == 0){
        PS3.setLedOff();
		PS3.setLedOn(LED1); //digital mode
    }
	else if(mode_Selector == 1){
        PS3.setLedOff();
		PS3.setLedOn(LED2); //analog mode
    }
	else{
		PS3.setLedOff();
		PS3.setLedOn(LED3); //Digital with digital stick mode
		
	}

}