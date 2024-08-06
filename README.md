# TTSpaceMouse

This is a small implementation of the TTSpaceMouse Pro Micro based DIY SpaceMouse.
As the josticks (4 of them) are quite in-accuarate as well as much slack before move is detected, I 
decided to make my own version where I am trying to detect "major moves" and decrease other detected
movements. The idea is to somewhat boost the "major detected" move in order to make it easier to use.

Feel free to use this code here for any kind of purpose (except blaming me for bad idea, bad code etc...)

All information of the "Teaching Tech Space Mouse" can be found here: https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix
Thanks to all people that have been involved in this project.

To compile:
- Tested on a SparkFun Pro Micro 16MHz/5V (ATmega 32U4).
- I am using PlatformIO under Vscode.
- My dev comp is Lunux-based, hence I have used the spacenavd project to test/develop under Linux. I do find that it works somewhat different
than the original Windows driver. Hence there is a section in the code that is disabled by default, this is for my testing purpose.
spacenavd can be found here: https://github.com/FreeSpacenav/spacenavd/

- There is a script-hook in the platoformio.ini to set the VID/PID. Without this, it will just be an ordinary HID device. With it, it will
look/behave like a 3Dconnexion device.

- My guess (not tested) is that if you like to test this using Arduino, follow the tutorial in the above project and simply copy&paste the
code into your project. (Do not miss the part where you customize the VID/PID)
