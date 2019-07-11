### Playstation Dual Shock 3/4 or Xbox One Controller to Sega Saturn adapter.




![Alt text](/DS4toSat.jpg?raw=true "DS4 to Saturn")

 

#### What it is
This lets you use a Dual Shock 3/4 or an Xbox One controller with your Sega Saturn.



#### How it works

This uses an Arduino and a USB shield to emulate the Sega Saturn controller, enabling you to use a Dual Shock 3/4 or Xbox One controller, or with a little editing to the source code, just about any controller that is compatible with the USB shield library.
You can toggle between analog and digital using the Options button on the DS4, Select on the DS3 and Menu button on the Xbox One. When the DS4 light is green it is set to analog, blue is digital and Magenta means the stick can be used digitally. The light would be red for 30 seconds on start-up if the battery is below 14% of its full charge.

There is of course no rumble support because the Saturn never supported rumble, but rudimentary support can be added eg. Say you are playing Quake you can add the vibrate code to the button mapped to the gun trigger to simulate recoil for that specific game. The button will always vibrate the controller though if pressed while not in game.

Being that the Saturn controller has six face buttons and the Dual Shock  and Xbox One has only four, the C button is mapped to R3 and Z to L3 on Dual Shock and Xbox One controller(not ideal for fighting games which the DS is a poor controller for anyway).

N.B. An external power source is needed for the Arduinos to power the Xbox One controller.

#### What's Needed

 - Arduino(I used the Arduino UNO rev 3)
 - USB Shield
 - Compatible bluetooth dongle(or you can use a usb cable)
 - Playstation 3/4 Dual Shock controller or Xbox One controller
 - Sega Saturn controller cable to connect the controller port to the Arduino


#### Putting it together

 - Pins A0-A3 on the Arduino is used for D0-D3 on the Saturn. Pin 3 on the Arduino is used for the TH line, pin 4 for the TR line, pin 5 is the 5v line and the ACK line is pin A4. DO NOT plug this into the Saturn as yet. See below.

|  Saturn | Arduino |
|---|---|
|D0 (Data 0)| A0 |
|D1 (Data 1)| A1 |
|D2 (Data 2)| A2 |
|D3 (Data 3)| A3 |
|Acknowledge/ACK (also known as TL)| A4 |
|Select/SEL (Select 0, also known as TH)| 3 |
|Request/REQ (Select 1, also known as TR)| 4 |
|5v (Power)| 5 |
|Ground| Ground |

 - Connect your shield and plug in your bluetooth dongle. Compile and flash your Arduino with the source code from here(don't forget to include the USB shield library).
 - Power on the Saturn and now plug the controller cable with the Arduino into the Saturn port.
 - Pair the controller with the Arduino.

 ##### Warning
 If powering the Arduino from an external source make sure the Saturn is on before supplying power to the Arduino. You risk damaging your Saturn otherwise.
There is some protection against this but response time can be as slow as 650us.



 #### Credit

Some of the code used here was built upon the previous work and help of user RDC @ [NFGgames.com](http://nfggames.com/forum2/index.php?topic=5055.msg33242#msg33242) Major props to him!!
Some minor code used from the USB Host Shield Library.


 ##### License
This is released under the GNU General Public License.

##### Contact info
I can be contacted at garybethelsgit@digitaltakesroot.com

