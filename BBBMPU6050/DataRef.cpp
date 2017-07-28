//Written by Dean Hovinghoff
#include "DataRef.h"


/****************************************************************
 * PHTGP(PinHeaderToGPIOPin
 ****************************************************************/
using namespace std;
int PHTGP(int Header, int Pin) //PinHeaderToGPIOPin
{
	if (Header == 8)
	{
		switch (Pin)
		{
			case 3:
				return 38;
			case 4:
				return 39;
			case 5:
				return 34;
			case 6:
				return 35;
			case 7:
				return 66;
			case 8:
				return 67;
			case 9:
				return 69;
			case 10:
				return 68;
			case 11:
				cout << "WARNINGsystem(CommandFinal.c_str: Use is not recommended USED By eMMC flash" << endl;
				return 45;
			case 12:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 44;
			case 13:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 23;
			case 14:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 26;
			case 15:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 47;
			case 16:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 46;
			case 17:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 27;
			case 18:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 65;
			case 19:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 22;
			case 20:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 63;
			case 21:
				cout << "WARNING: Use is not recommended USED By eMMC flash" << endl;
				return 62;
			case 22:
				return 37;
			case 23:
				return 36;
			case 24:
				return 33;
			case 25:
				return 32;
			case 26:
				return 61;
			case 27:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 86;
			case 28:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 88;
			case 29:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 87;
			case 30:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 89;
			case 31:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 10;
			case 32:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 11;
			case 33:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 9;
			case 34:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 81;
			case 35:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 8;
			case 36:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 80;
			case 37:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 78;
			case 38:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 79;
			case 39:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 76;
			case 40:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 77;
			case 41:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 74;
			case 42:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 75;
			case 43:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 72;
			case 44:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 73;
			case 45:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 70;
			case 46:
				//cout << "WARNING: Use is not recommended USED By HDMI i/f" << endl;
				return 71;
			default:
				cout << "ERROR: Invalid pin" << endl;
				return 0;
		}
	} else if (Header == 9)
	{
		switch (Pin)
		{
			case 11:
				return 30;
			case 12:
				return 60;
			case 13:
				return 31;
			case 14:
				return 50;
			case 15:
				return 48;
			case 16:
				return 51;
			case 17:
				return 5;
			case 18:
				return 4;
			case 19:
				return 13;
			case 20:
				return 12;
			case 21:
				return 3;
			case 22:
				return 2;
			case 23:
				return 49;
			case 24:
				return 15;
			case 25:
				return 117;
			case 26:
				return 14;
			case 27:
				return 115;
			case 28:
				return 113;
			case 29:
				return 111;
			case 30:
				return 112;
			case 31:
				return 110;
			default:
				cout << "ERROR: Invalid pin" << endl;
				return 0;
		}
	} else
	{
		cout << "ERROR: not valid Header";
	}
	return 0;
}


