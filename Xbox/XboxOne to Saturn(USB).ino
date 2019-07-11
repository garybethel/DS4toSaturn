/*
 Example sketch for the Xbox ONE USB library - by guruthree, based on work by
 Kristian Lauszus.
 */

#include <XBOXONE.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXONE Xbox(&Usb);

#define dead_zone 990  //Dead area of analog stick
#define analog_digital_dead_zone 1486
uint8_t mode_Selector = 0;  //0 is digital mode 1 is analog

unsigned char DATA1_1, DATA1_2;
unsigned char DATA2_1, DATA2_2;
unsigned char DATA3_1, DATA3_2;
unsigned char DATA4_1, DATA4_2;
unsigned char DATA5_1, DATA5_2;
unsigned char DATA6_1, DATA6_2;
unsigned char DATAEND_1, DATAEND_2;

bool ACK_LINE = 0;
bool battery_status = false;
bool power_on = false;
long battery_timer_start;


void setup() {

 DATA1_1 = B00001111;                 // DATA1_1
 DATA1_2 = B00001111;                 // DATA1_2
 DATA2_1 = B00001111;                 // DATA2_1
 DATA2_2 = B00001111;                 // DATA2_2
 DATA3_1 = B00000111;                 // DATA3_1
 DATA3_2 = B00001111;                 // DATA3_2 
 DATA4_1 = B00000111;                 // DATA4_1
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
 Serial.print(F("\r\nPS4 USB Library Started")); 
 
 
 if(managePowerState()){
	//pull the ack line high on startup
	changeACKState();
 }
 //we need to wait for the saturn to startup, 1.5 seconds
 delayMicroseconds(500);
 battery_timer_start = millis();
 }

 void loop() {

	Usb.Task();
	power_on = managePowerState(); //handles powering on and off the saturn
	
	//only if power is on do we want to run this routine
	if(power_on){
		if (Xbox.XboxOneConnected) {
			
			//Changes the controller from Digital to Analog
			if (Xbox.getButtonClick(START)){
				if(mode_Selector == 0){
					mode_Selector = 1; //change state to now operating in analog mode
					Serial.print(F("\r\nController switched to Analog"));
				}
				else{
					mode_Selector = 0; //change state to now operating in digital mode
					Serial.print(F("\r\nController switched to Digital"));
				}
			}
			if(mode_Selector == 0){
				emulateDigitalController(); 
			}
			else if(mode_Selector == 1) {
				emulateAnalogController();
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
	if(readS0()==0){
		delayMicroseconds(40);
		//waiting for the saturn
		while(readS1()==1 && readS0()==0){
		}
	
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC =  B00000000;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		
		while(readS1()==0 && readS0()==0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B00000010;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		//// START SECOND BYTE
		
		//Wait for Saturn to request first half of Data2
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA2_2;
		changeACKState();
		//// END SECOND BYTE
		
		//// START THIRD BYTE, END OF DATA
		
		// Wait for Saturn to request first half End
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATAEND_1;
		changeACKState();
		// Wait for Saturn to request second half of End
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		/// END THIRD BYTE
		// Wait for the Saturn to deselect controller	
		//This wait is 104us and was removed so that we begin to poll the controller input earlier. 
		//while(readS0()==0){}
		
		DATA1_1 = B00001111;
		DATA1_2 = B00001111;
		DATA2_1 = B00001111;
		DATA2_2 = B00001111;

		
			//If the Up button pressed
		if(Xbox.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(Xbox.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(Xbox.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(Xbox.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		//If the B button pressed
		if(Xbox.getButtonPress(B)){DATA1_2 &= B00001110;}
		//If the C button pressed
		if(Xbox.getButtonPress(R3)){DATA1_2 &= B00001101;}
		//If the A button pressed
		if(Xbox.getButtonPress(A)){DATA1_2 &= B00001011;}
		//If the Start button pressed
		if (Xbox.getButtonPress(XBOX)){DATA1_2 &= B00000111;}
		
		//If the Z button pressed
		if(Xbox.getButtonPress(L3)){DATA2_1 &= B00001110;}
		//If the Y button pressed
		if(Xbox.getButtonPress(Y)){DATA2_1 &= B00001101;}
		//If the X button pressed
		if(Xbox.getButtonPress(X)){DATA2_1 &= B00001011;}
		//If the Right trigger button pressed
		if(Xbox.getButtonPress(R1)){DATA1_2 &= B00001101;}
		
		//If the Left trigger button pressed
		if(Xbox.getButtonPress(L1)){DATA2_1 &= B00001110;}
		
		
		int16_t analogReading;
		analogReading = Xbox.getAnalogHat(LeftHatX);
		if(analogReading >= (7500+analog_digital_dead_zone) || analogReading <= (-7500 - analog_digital_dead_zone)) {
			//Right
			if(analogReading >= 7500 + analog_digital_dead_zone){
				DATA1_1 &= B00000111;
			}
			//Left
			if(analogReading <= -7500 - analog_digital_dead_zone){
				DATA1_1 &= B00001011;
			}
		}
		
		analogReading = Xbox.getAnalogHat(LeftHatY);
		if(analogReading >= (7500 + analog_digital_dead_zone) || analogReading <= (-7500 - analog_digital_dead_zone)) {
			//Up
			if(analogReading <= -7500 - analog_digital_dead_zone){
				DATA1_1 &= B00001101;
			}
			//Down
			if(analogReading >= 7500 + analog_digital_dead_zone){
				DATA1_1 &= B00001110;
			}
		}
	
		delayMicroseconds(28);//added since we need to be at 104us wait average
	}
	
}
//this entire function takes 650 us once controller s0 goes low
void emulateAnalogController(){

	////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0()==0){
		delayMicroseconds(40);
		
		//waiting for the saturn
		while(readS1()==1 && readS0()==0){ 
		}
		
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC = B00000001;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		while(readS1()==0 && readS0()==0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B0000110;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		
		///// START SECOND BYTE
		//Wait for Saturn to request first half of Data2
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA2_2;
		changeACKState();
		
		///// END SECOND BYTE
		
		///// START THIRD BYTE,
		
		// Wait for Saturn to request first half of Data3
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA3_1;
		changeACKState();
		// Wait for Saturn to request second half of Data3
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA3_2;
		changeACKState();
		//Usb.Task(); //new	
		//// END THIRD BYTE
		
		///// START FOURTH BYTE,
		
		// Wait for Saturn to request first half of Data4
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA4_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data4
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA4_2;
		changeACKState();
		///// END FOURTH BYTE
		
		///// START FIFTH BYTE,
		
		// Wait for Saturn to request first half of Data5
		while(readS0()==0 && readS1()==1){
		} 
		PORTC = DATA5_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data5
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA5_2;
		changeACKState();
		
		///// END FIFTH BYTE

		///// START SIXTH BYTE,
		
		// Wait for Saturn to request first half of Data6
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA6_1;
		changeACKState();
		// Wait for Saturn to request second half of Data6
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA6_2;
		changeACKState();
		///// END SIXTH BYTE
		
		///// START SEVENTH BYTE, END OF DATA
		// Wait for Saturn to request first half End
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATAEND_1;
		changeACKState();
		
		// Wait for Saturn to request second half of End
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		///// END SEVENTH BYTE
		
		// Wait for the Saturn to deselect controller
		/*while(readS0()==0){
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
		 
		int16_t analogReading;
		
		//If the Up button pressed
		if(Xbox.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(Xbox.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(Xbox.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(Xbox.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		//If the B button pressed
		if(Xbox.getButtonPress(B)){DATA1_2 &= B00001110;}
		//If the C button pressed
		if(Xbox.getButtonPress(R3)){DATA1_2 &= B00001101;}
		//If the A button pressed
		if(Xbox.getButtonPress(A)){DATA1_2 &= B00001011;}
		//If the Start button pressed
		if (Xbox.getButtonPress(XBOX)){DATA1_2 &= B00000111;}
		
		//If the Z button pressed
		if(Xbox.getButtonPress(L3)){DATA2_1 &= B00001110;}
		//If the Y button pressed
		if(Xbox.getButtonPress(Y)){DATA2_1 &= B00001101;}
		//If the X button pressed
		if(Xbox.getButtonPress(X)){DATA2_1 &= B00001011;}
		//If the Right trigger button pressed
		if(Xbox.getButtonPress(R1)){DATA1_2 &= B00001101;}
		
		//If the Left trigger button pressed
		if(Xbox.getButtonPress(L1)){DATA2_1 &= B00001110;}
			
		//Data3
		analogReading = Xbox.getAnalogHat(LeftHatX);
		if(analogReading > (7500 + dead_zone) || analogReading < (-7500 - dead_zone)) {
			analogReading = map(analogReading, -32767, 32767, 0, 255);
			getBinary(analogReading,DATA3_1,DATA3_2);
		}
		
		//Data4
		analogReading = Xbox.getAnalogHat(LeftHatY);
		if(analogReading > (7500 + dead_zone) || analogReading < (-7500 - dead_zone)) {
			analogReading = 255 - map(analogReading, -32767, 32767, 0, 255);//we subtract 255 because we need to reverse the reading
			getBinary(analogReading,DATA4_1,DATA4_2);
		}
		//Data5
		analogReading = Xbox.getButtonPress(R2);
		if(analogReading > 0) {
			analogReading = 255 - map(analogReading, 0, 1023, 0, 255);
			getBinary(analogReading,DATA5_1,DATA5_2);
		}
			
		//Data6
		analogReading = Xbox.getButtonPress(L2);
		if(analogReading > 0) {
			analogReading = 255 - map(analogReading, 0, 1023, 0, 255);
			getBinary(analogReading,DATA6_1,DATA6_2);
		}

	}	
}


//use to determine the status of the ACK return line and send back the opposite.eg. If its low now then return a high;
 void changeACKState(){
  if(ACK_LINE==0){
	PORTC|=B00010000;
	ACK_LINE =1;
  }
  else{
	PORTC &=~ B00010000;
	ACK_LINE =0;
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
	temp= char(num);
	second_nibble = (second_nibble & 0b11110000) | (temp & 0b00001111);
	temp = temp>>4;
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
