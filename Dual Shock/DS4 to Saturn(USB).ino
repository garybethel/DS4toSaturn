#include <PS4USB.h>
#include <SPI.h>

USB Usb;
PS4USB PS4(&Usb);


/*DEADZONE*/#define dead_zone 10//  Dead area of analog stick
#define analog_digital_dead_zone 18

#define pin_In_S0 3 //th
#define pin_In_S1 4 //tr
#define pin_Out_Y0 A0
#define pin_Out_Y1 A1
#define pin_Out_Y2 A2
#define pin_Out_Y3 A3
#define pin_ACK_Y4 A4 //ack line

char pin_Array[4]{A0,A1,A2,A3};

uint8_t mode_selector = 0;  //0 is digital mode 1 is analog
bool ack_line = 0;
bool battery_status = false;
long battery_timer_start;
bool retain_trigger_position = false;

uint8_t controller_data[14]{};


void setup() {
	Serial.begin(115200);

    DefaultControllerData();

    pinMode(pin_In_S0, INPUT);
    pinMode(pin_In_S1, INPUT);
    pinMode(pin_Out_Y0, OUTPUT);
    pinMode(pin_Out_Y1, OUTPUT);
    pinMode(pin_Out_Y2, OUTPUT);
    pinMode(pin_Out_Y3, OUTPUT);
    pinMode(pin_ACK_Y4, OUTPUT);

    #if !defined(__MIPSEL__)
      while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    #endif
    if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start"));
      while (1); // Halt
    }
    Serial.print(F("\r\nPS4 USB Library Started")); 
    delay(200);

    
    //pull the ack line high on startup
    changeACKState();
    
     //we need to wait for the saturn to startup, 1.5 seconds
     //delayMicroseconds(500);
     battery_timer_start = millis();
 }

 void loop() {

 	Usb.Task();
 
	if (PS4.connected()) {
		
		checkBatteryStatus();
		//Changes the controller from Digital to Analog
		if (PS4.getButtonClick(OPTIONS)){
			if(mode_selector == 0){
				mode_selector = 1; //change state to now operating in analog mode
				Serial.print(F("\r\nController switched to Analog"));
			}
			else if(mode_selector == 1){
				mode_selector = 2; //change state to now operating in Twin Stick mode
				Serial.print(F("\r\nController switched to Twin Sticks"));
			}
			else{
				mode_selector = 0; //change state to now operating in digital mode
				Serial.print(F("\r\nController switched to Digital"));
			}
		}
		
		if(mode_selector == 0){
			emulateDigitalController(); 
			setLedColor(); //led would change to blue
		}
		else if(mode_selector == 1) {
			emulateAnalogController();
			setLedColor(); //led would change to green
		}
		else{
			emulateDigitalController();//With functional sticks
			setLedColor(); //led would change to magenta
		}
	}
	else{
		sendDataStateHigh(); //if no PS controller is attached we send the data lines to high emulating no controller for the Saturn
		Serial.print(F("\r\not connected"));
	}
 }

void emulateDigitalController(){
	
////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0() == 0){  //s1-tr, s0-th
	
		delayMicroseconds(40);
		//waiting for the saturn
		while(readS1() == 1 && readS0() == 0){}
		
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		sendToController(B00000000);
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		
		while(readS1() == 0 && readS0() == 0){}
		
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		sendToController(B00000010);
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1() == 1 && readS0() == 0){}

		sendToController(controller_data[0]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1() == 0 && readS0() == 0){}

		sendToController(controller_data[1]);
		changeACKState();
		///// END FIRST BYTE
		
		//// START SECOND BYTE
		
		//Wait for Saturn to request first half of Data2
		while(readS1() == 1 && readS0() == 0){}

		sendToController(controller_data[2]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1() == 0 && readS0() == 0){}

		sendToController(controller_data[3]);
		changeACKState();
		//// END SECOND BYTE
		
		//// START THIRD BYTE, END OF DATA
		
		// Wait for Saturn to request first half End
		while(readS1() == 1 && readS0() == 0){}

		sendToController(controller_data[12]);
		changeACKState();
		// Wait for Saturn to request second half of End
		while(readS1() == 0 && readS0() == 0){}
		
		sendToController(controller_data[13]);
		changeACKState();
		
		/// END THIRD BYTE
		// Wait for the Saturn to deselect controller	
	
		//if(PS4.connected()){
			DefaultControllerData();
			
			uint8_t analogReading;
			
			///Twin stick instructions
			if(mode_selector == 2){
				
				if (PS4.getButtonPress(PS)){controller_data[1] &= B00000111;}
				
				analogReading = PS4.getAnalogHat(LeftHatX);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Right
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[0] &= B00000111;}
					//Left
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[0] &= B00001011;}
				}
				
				analogReading = PS4.getAnalogHat(LeftHatY);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Up
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[0] &= B00001110;}
					//Down
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[0] &= B00001101;}
				}
				
				////////////////////////////////////
				analogReading = PS4.getAnalogHat(RightHatX);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Z for right
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[2] &= B00001110;}
					//X for left
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[2] &= B00001011;}
				}
				
				analogReading = PS4.getAnalogHat(RightHatY);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Y for up
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[2] &= B00001101;}
					//B for down
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[1] &= B00001110;}
				}
				//C right dash button
				if(PS4.getAnalogButton(R2) >0) {controller_data[1] &= B00001101;}
				//R Left dash button
				if(PS4.getAnalogButton(L2) >0) {controller_data[2] &= B00000111;}
				//A for right side weapons
				if(PS4.getButtonPress(R1)) {controller_data[1] &= B00001011;}
					
				//L left side weapons
				if(PS4.getButtonPress(L1)) {controller_data[3] &= B00000111;}
				
			}//Digital controller 
			else{
				
				//If the Up button pressed
				if(PS4.getButtonPress(UP)){controller_data[0] &= B00001110;}
				//If the Down button pressed
				if(PS4.getButtonPress(DOWN)){controller_data[0] &= B00001101;}
				//If the Left button pressed
				if(PS4.getButtonPress(LEFT)){controller_data[0] &= B00001011;}
				//If the Right button pressed
				if(PS4.getButtonPress(RIGHT)){controller_data[0] &=B00000111;}
				
				/*CIRCLE*/if(PS4.getButtonPress(CIRCLE)){controller_data[1] &= B00001110;}/*CIRCLE*/
				/*R3*/if(PS4.getButtonPress(R3)){/**/}/*R3*/
				/*CROSS*/if(PS4.getButtonPress(CROSS)){controller_data[1] &= B00001011;}/*CROSS*/
				
				//If the Start button pressed
				if (PS4.getButtonPress(PS)){controller_data[1] &= B00000111;}
				
				/*L3*/if(PS4.getButtonPress(L3)){/**/}/*L3*/
				/*TRIANGLE*/if(PS4.getButtonPress(TRIANGLE)){controller_data[2] &= B00001101;}/*TRIANGLE*/
				/*SQUARE*/if(PS4.getButtonPress(SQUARE)){controller_data[2] &= B00001011;}/*SQUARE*/
				/*R1*/if(PS4.getButtonPress(R1)){controller_data[1] &= B00001101;}/*R1*/
				/*L1*/if(PS4.getButtonPress(L1)){controller_data[2] &= B00001110;}/*L1*/
				
				if(PS4.getAnalogButton(R2) >0) {controller_data[2] &= B00000111;}
				if(PS4.getAnalogButton(L2) >0) {controller_data[3] &= B00000111;}
				

				////////stick function assignment 
				analogReading = PS4.getAnalogHat(LeftHatX);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Right
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[0] &= B00000111;}
					//Left
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[0] &= B00001011;}
				}
				analogReading = PS4.getAnalogHat(LeftHatY);
				if(analogReading >= (127 + analog_digital_dead_zone) || analogReading <= (127 - analog_digital_dead_zone)) {
					//Up
					if(analogReading <= 127 - analog_digital_dead_zone){controller_data[0] &= B00001110;}
					//Down
					if(analogReading >= 127 + analog_digital_dead_zone){controller_data[0] &= B00001101;}
				}
				
			}
			//delayMicroseconds(18);//added since we need to be at 104us wait average..........out of spec but minused 10us from the original time 
		//}
		//else{
			while(readS0() == 0){}
		//}
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
		while(readS1() == 1 && readS0() == 0){}
		
		
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		sendToController(B00000001);
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		while(readS1() == 0 && readS0() == 0){}
		
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		sendToController(B0000110);
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1() == 1 && readS0() == 0){}
		
		sendToController(controller_data[0]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1() == 0 && readS0() == 0){}
		
		sendToController(controller_data[1]);
		changeACKState();
		///// END FIRST BYTE
		
		///// START SECOND BYTE
		//Wait for Saturn to request first half of Data2
		while(readS1() == 1 && readS0() == 0){}
		
		sendToController(controller_data[2]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1() == 0 && readS0() == 0){}
		
		sendToController(controller_data[3]);
		changeACKState();
		
		///// END SECOND BYTE
		
		///// START THIRD BYTE,
		
		// Wait for Saturn to request first half of Data3
		while(readS0() == 0 && readS1() == 1){}
		
		sendToController(controller_data[4]);
		changeACKState();
		// Wait for Saturn to request second half of Data3
		while(readS0() == 0 && readS1() == 0){}
		
		sendToController(controller_data[5]);
		changeACKState();

		//// END THIRD BYTE
		
		///// START FOURTH BYTE,
		
		// Wait for Saturn to request first half of Data4
		while(readS0() == 0 && readS1() == 1){}
		
		sendToController(controller_data[6]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data4
		while(readS0() == 0 && readS1() == 0){}
		
		sendToController(controller_data[7]);
		changeACKState();
		///// END FOURTH BYTE
		
		///// START FIFTH BYTE,
		
		// Wait for Saturn to request first half of Data5
		while(readS0() == 0 && readS1() == 1){}
		
		sendToController(controller_data[8]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data5
		while(readS0() == 0 && readS1() == 0){}
		
		sendToController(controller_data[9]);
		changeACKState();
		
		///// END FIFTH BYTE

		///// START SIXTH BYTE,
		
		// Wait for Saturn to request first half of Data6
		while(readS0() == 0 && readS1() == 1){}
		
		sendToController(controller_data[10]);
		changeACKState();
		// Wait for Saturn to request second half of Data6
		while(readS0() == 0 && readS1() == 0){}
		
		sendToController(controller_data[11]);
		changeACKState();
		///// END SIXTH BYTE
		
		///// START SEVENTH BYTE, END OF DATA
		// Wait for Saturn to request first half End
		while(readS0() == 0 && readS1() == 1){}
		
		sendToController(controller_data[12]);
		changeACKState();
		
		// Wait for Saturn to request second half of End
		while(readS0() == 0 && readS1() == 0){}
		
		sendToController(controller_data[13]);
		changeACKState();
		
		///// END SEVENTH BYTE
		
		// Wait for the Saturn to deselect controller
		/*while(readS0()==0){
		}*/
		
		///// END COMMS WITH SATURN
		
		DefaultControllerData(); 
		
		uint8_t analogReading;
		uint8_t adjusted_AnalogReading;
		
		
		//If the Up button pressed
		if(PS4.getButtonPress(UP)){controller_data[0] &= B00001110;}
		//If the Down button pressed
		if(PS4.getButtonPress(DOWN)){controller_data[0] &= B00001101;}
		//If the Left button pressed
		if(PS4.getButtonPress(LEFT)){controller_data[0] &= B00001011;}
		//If the Right button pressed
		if(PS4.getButtonPress(RIGHT)){controller_data[0] &=B00000111;}
		
		/*CIRCLE*/if(PS4.getButtonPress(CIRCLE)){controller_data[1] &= B00001110;}/*CIRCLE*/
		/*R3*/if(PS4.getButtonPress(R3)){/**/}/*R3*/
		/*CROSS*/if(PS4.getButtonPress(CROSS)){controller_data[1] &= B00001011;}/*CROSS*/
		
		//If the Start button pressed
		if (PS4.getButtonPress(PS)){controller_data[1] &= B00000111;}
		
		/*L3*/if(PS4.getButtonPress(L3)){/**/}/*L3*/
		/*TRIANGLE*/if(PS4.getButtonPress(TRIANGLE)){controller_data[2] &= B00001101;}/*TRIANGLE*/
		/*SQUARE*/if(PS4.getButtonPress(SQUARE)){controller_data[2] &= B00001011;}/*SQUARE*/

		/*R1*/if(PS4.getButtonPress(R1)){controller_data[1] &= B00001101;}/*R1*/  //C
		/*L1*/if(PS4.getButtonPress(L1)){controller_data[2] &= B00001110;}/*L1*/ //Z
		
		//Data3
		analogReading = PS4.getAnalogHat(LeftHatX);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,controller_data[4],controller_data[5]);
		}
		//Data4
		analogReading = PS4.getAnalogHat(LeftHatY);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,controller_data[6],controller_data[7]);
		}
		//Data5
		analogReading = PS4.getAnalogButton(L2);
		if(analogReading > 0) {
			adjusted_AnalogReading = (255 - analogReading);
			getBinary(adjusted_AnalogReading,controller_data[8],controller_data[9]);
			
			//in the event the controller uses digital triggers in analog mode
			if(analogReading > 144 || (retain_trigger_position && analogReading > 86)){
				controller_data[3] &= B00000111;
				retain_trigger_position = true;
			}
			else if(analogReading < 86) {
				retain_trigger_position = false;
			}
		}
		
		//Data6
		analogReading = PS4.getAnalogButton(R2);
		if(analogReading > 0) {
			adjusted_AnalogReading = (255 - analogReading);
			getBinary(adjusted_AnalogReading,controller_data[10],controller_data[11]);
			
			//in the event the controller uses digital triggers in analog mode
			if(analogReading > 144 || (retain_trigger_position && analogReading > 86)){
				controller_data[2] &= B00000111;
				retain_trigger_position = true;
			}
			else if(analogReading < 86) {
				retain_trigger_position = false;
			}
		}
		while(readS0() == 0){}
	}	
}

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
		sendToController(B00000001);
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		while(readS1() == 0 && readS0() == 0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		sendToController(B0000110);
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1() == 1 && readS0() == 0){
		}
		sendToController(controller_data[0]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1() == 0 && readS0() == 0){
		}
		sendToController(controller_data[1]);
		changeACKState();
		///// END FIRST BYTE
		
		///// START SECOND BYTE
		//Wait for Saturn to request first half of Data2
		while(readS1() == 1 && readS0() == 0){
		}
		sendToController(controller_data[2]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1() == 0 && readS0() == 0){
		}
		sendToController(controller_data[3]);
		changeACKState();
		
		///// END SECOND BYTE
		
		///// START THIRD BYTE,
		
		// Wait for Saturn to request first half of Data3
		while(readS0() == 0 && readS1() == 1){
		}
		sendToController(controller_data[4]);
		changeACKState();
		// Wait for Saturn to request second half of Data3
		while(readS0() == 0 && readS1() == 0){
		}
		sendToController(controller_data[5]);
		changeACKState();

		//// END THIRD BYTE
		
		///// START FOURTH BYTE,
		
		// Wait for Saturn to request first half of Data4
		while(readS0() == 0 && readS1() == 1){
		}
		sendToController(controller_data[6]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data4
		while(readS0() == 0 && readS1() == 0){
		}
		sendToController(controller_data[7]);
		changeACKState();
		///// END FOURTH BYTE
		
		///// START FIFTH BYTE,
		
		// Wait for Saturn to request first half of Data5
		while(readS0() == 0 && readS1() == 1){
		} 
		sendToController(controller_data[8]);
		changeACKState();
		
		// Wait for Saturn to request second half of Data5
		while(readS0() == 0 && readS1() == 0){
		}
		sendToController(controller_data[9]);
		changeACKState();
		
		///// END FIFTH BYTE

		///// START SIXTH BYTE,
		
		// Wait for Saturn to request first half of Data6
		while(readS0() == 0 && readS1() == 1){
		}
		sendToController(controller_data[10]);
		changeACKState();
		// Wait for Saturn to request second half of Data6
		while(readS0() == 0 && readS1() == 0){
		}
		sendToController(controller_data[11]);
		changeACKState();
		///// END SIXTH BYTE
		
		///// START SEVENTH BYTE, END OF DATA
		// Wait for Saturn to request first half End
		while(readS0() == 0 && readS1() == 1){
		}
		sendToController(controller_data[12]);
		changeACKState();
		
		// Wait for Saturn to request second half of End
		while(readS0() == 0 && readS1() == 0){
		}
		sendToController(controller_data[13]);
		changeACKState();
		
		///// END SEVENTH BYTE
		
		// Wait for the Saturn to deselect controller
		/*while(readS0()==0){
		}*/
		
		///// END COMMS WITH SATURN
		
		DefaultControllerData(); 
		
		uint8_t analogReading;
		uint8_t adjusted_AnalogReading;
		
		
		//If the Up button pressed
		if(PS4.getButtonPress(UP)){controller_data[0] &= B00001110;}
		//If the Down button pressed
		if(PS4.getButtonPress(DOWN)){controller_data[0] &= B00001101;}
		//If the Left button pressed
		if(PS4.getButtonPress(LEFT)){controller_data[0] &= B00001011;}
		//If the Right button pressed
		if(PS4.getButtonPress(RIGHT)){controller_data[0] &=B00000111;}
		
		/*CIRCLE*/if(PS4.getButtonPress(CIRCLE)){controller_data[1] &= B00001110;}/*CIRCLE*/
		/*R3*/if(PS4.getButtonPress(R3)){/**/}/*R3*/
		/*CROSS*/if(PS4.getButtonPress(CROSS)){controller_data[1] &= B00001011;}/*CROSS*/
		
		//If the Start button pressed
		if (PS4.getButtonPress(PS)){controller_data[1] &= B00000111;}
		
		/*L3*/if(PS4.getButtonPress(L3)){/**/}/*L3*/
		/*TRIANGLE*/if(PS4.getButtonPress(TRIANGLE)){controller_data[2] &= B00001101;}/*TRIANGLE*/
		/*SQUARE*/if(PS4.getButtonPress(SQUARE)){controller_data[2] &= B00001011;}/*SQUARE*/

		/*R1*/if(PS4.getButtonPress(R1)){controller_data[1] &= B00001101;}/*R1*/  //C
		/*L1*/if(PS4.getButtonPress(L1)){controller_data[2] &= B00001110;}/*L1*/ //Z
		
		//Data3
		analogReading = PS4.getAnalogHat(LeftHatX);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,controller_data[4],controller_data[5]);
		}
		//Data4
		analogReading = PS4.getAnalogHat(LeftHatY);
		if(analogReading >= (127 + dead_zone) || analogReading <= (127 - dead_zone)) {
			getBinary(analogReading,controller_data[6],controller_data[7]);
		}
		//Data5
		analogReading = PS4.getAnalogButton(L2);
		if(analogReading > 0) {
			adjusted_AnalogReading = (255 - analogReading);
			getBinary(adjusted_AnalogReading,controller_data[8],controller_data[9]);
			
			//in the event the controller uses digital triggers in analog mode
			if(analogReading > 144 || (retain_trigger_position && analogReading > 86)){
				controller_data[3] &= B00000111;
				retain_trigger_position = true;
			}
			else if(analogReading < 86) {
				retain_trigger_position = false;
			}
		}
		
		//Data6
		analogReading = PS4.getAnalogButton(R2);
		if(analogReading > 0) {
			adjusted_AnalogReading = (255 - analogReading);
			getBinary(adjusted_AnalogReading,controller_data[10],controller_data[11]);
			
			//in the event the controller uses digital triggers in analog mode
			if(analogReading > 144 || (retain_trigger_position && analogReading > 86)){
				controller_data[2] &= B00000111;
				retain_trigger_position = true;
			}
			else if(analogReading < 86) {
				retain_trigger_position = false;
			}
		}
		while(readS0() == 0){}
	}	
}


//use to determine the status of the ACK return line and send back the opposite.eg. If its low now then return a high;
void changeACKState(){
	if(ack_line == 0){
	  	digitalWrite(A4,1);
		ack_line = 1;
	}
	else{
		digitalWrite(A4,0);
		ack_line = 0;
	}
 }


void getBinary(uint8_t num, uint8_t &first_nibble, uint8_t &second_nibble){
 
	second_nibble = (second_nibble & 0b11110000) | (num & 0b00001111);
	first_nibble = (first_nibble & 0b11110000) | ( (num >>4) & 0b00001111);
}
 
bool readS0(){
    return(digitalRead(pin_In_S0));
}

bool readS1(){
    return(digitalRead(pin_In_S1));
}

void sendDataStateHigh(){
	sendToController(B00001111);
}

void sendToController(char in){

	for (int i = 0; i < 4; i++){
		int pin_state = (in >> i) & 1;

      digitalWrite(pin_Array[i], pin_state);
        
	}
}

//checks the battery status and sets led red for 1/2 a minute if battery is less than 14% of its full charge
void checkBatteryStatus(){
    if(battery_status == false){
        if(PS4.getBatteryLevel() < 2){
            long battery_timer_elapsed = millis() - battery_timer_start;
            if(battery_timer_elapsed >= 30000){
                battery_status = true;
            }
        }
        else{
            battery_status = true;
        }
    }
}


void setLedColor(){

	if(battery_status == false){
	    PS4.setLed(Red); //battery weak
	}
	else if(mode_selector == 0){
	    PS4.setLed(Blue); //digital mode
	}
	else if(mode_selector == 1){
		PS4.setLed(Green); //analog mode
	}
	else if(mode_selector == 2){
	    PS4.setLed(255,0,255);// Twin Stick mode
	}

}


void DefaultControllerData(){

	controller_data[0] = B00001111; 
	controller_data[1] = B00001111;  
	controller_data[2] = B00001111;  
	controller_data[3] = B00001111;  
	controller_data[4] = B00000111;  
	controller_data[5] = B00001111;  
	controller_data[6] = B00000111;  
	controller_data[7] = B00001111;  
	controller_data[8] = B00001111;  
	controller_data[9] = B00001111;  
	controller_data[10] = B00001111;  
	controller_data[11] = B00001111;  
	controller_data[12] = B00000000;
	controller_data[13] = B00000001;
}
  


