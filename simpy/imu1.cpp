#include <stdio.h>
#include <arduinotalker.h>

int main()
{
	ArduinoTalker Romeo;
	Romeo = new ArduinoTalker(this);
	Romeo->ConnectToArduino("/dev/ttyACM0", errorLimit);
}
