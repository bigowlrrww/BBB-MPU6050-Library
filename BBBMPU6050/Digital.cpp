//Written by Dean Hovinghoff
#include "Digital.h"
using namespace std;
/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, PIN_DIRECTION out_flag)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}

	if (out_flag == OUTPUT_PIN)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, PIN_VALUE value)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if (value==LOW)
		write(fd, "0", 2);
	else
		write(fd, "1", 2);

	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}

	close(fd);
	return 0;
}
int gpio_get_value(unsigned int gpio)
{
	int fd;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') {
		close(fd);
		return 1;
	} else {
		close(fd);
		return 0;
	}
}
/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	write(fd, edge, strlen(edge) + 1);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
	return close(fd);
}

/****************************************************************
 * gpio_omap_mux_setup - Allow us to setup the ompat mux mode for a pin
 ****************************************************************/

int gpio_omap_mux_setup(const char *omap_pin0_name, const char *mode)
{
	int fd;
	char buf[MAX_BUF];
	snprintf(buf, sizeof(buf), SYSFS_OMAP_MUX_DIR "%s", omap_pin0_name);
	fd = open(buf, O_WRONLY);
	if (fd < 0)
	{
		perror("failed to open OMAP_MUX");
		return fd;
	}
	write(fd, mode, strlen(mode) + 1);
	close(fd);
	return 0;
}
/****************************************************************
 * digitalWrite
 ****************************************************************/
void digitalWrite(char *LED_ID,char v[])
{
	FILE *LEDHandle = NULL;

	if((LEDHandle = fopen(LED_ID, "r+")) != NULL) //Open file by var
	{
	fwrite(v, sizeof(char), 1, LEDHandle); //write the value to 0 'false'
	fclose(LEDHandle);     	//Close file after writing always!!
	}else{
	cout << "ERROR file could not be opened" << endl;
	}
}

/****************************************************************
 * adc_export
 ****************************************************************/
void adc_export()
{
	int fd;
        char buf[MAX_BUF];
        char ch[7];
        snprintf(buf, sizeof(buf), "/sys/devices/bone_capemgr.9/slots");
        fd = open(buf, O_WRONLY);
        if (fd < 0) 
	{
                perror("adc/activate-adc");
        }
        write(fd, "BB-ADC", 7);
	close(fd);
}

/****************************************************************
 * adc_get_value
 ****************************************************************/
int adc_get_value(unsigned int AIN)
{
	int fd;
	char buf[MAX_BUF];
	char ch[7];
	//snprintf(buf, sizeof(buf), "/sys/devices/bone_capemgr.9/slots");
	//fd = open(buf, O_WRONLY);
	//if (fd < 0) {
			//perror("adc/activate-adc");
			//return fd;
		//}
	//write(fd, "BB-ADC", 7);
	//close(fd);

	snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", AIN);
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("adc/get-value");
		return fd;
	}

	read(fd, ch, 6);
	close(fd);
	return atoi(ch);
}

/****************************************************************
 * getkey
 ****************************************************************/
char getkey()
{
	const int ROWS = 4;
	const int COLS = 4;
	char keys[ROWS][COLS] = {
			{'1','2','3','A'},
			{'4','5','6','B'},
			{'7','8','9','C'},
			{'*','0','#','D'}
	};

	int colPins[COLS] = {77, 75, 73, 71};
	int rowPins[ROWS] = {76, 74, 72, 70};
	int rowRead[ROWS] = {1, 1, 1, 1};
	//activity
	int e = 1; //ESCAPE INT FOR DEBUG set to 0 to excape
	for(int i = 0; i < ROWS; i++)
	{
		gpio_export(rowPins[i]);
		gpio_set_dir(rowPins[i], INPUT_PIN);
	}
	for(int i = 0; i < COLS; i++)
	{
		gpio_export(colPins[i]);
		gpio_set_dir(colPins[i], OUTPUT_PIN);
		gpio_set_value(colPins[i], HIGH);
	}


	while (e)
	{
		//Read values and get key
		for (int c = 0; c < COLS; c++)
		{
			gpio_set_value(colPins[c], LOW);
				rowRead[0] = gpio_get_value(rowPins[0]);
				rowRead[1] = gpio_get_value(rowPins[1]);
				rowRead[2] = gpio_get_value(rowPins[2]);
				rowRead[3] = gpio_get_value(rowPins[3]);
			for (int i = 0; i < ROWS; i++)
			{
				if(rowRead[i] == 0)
				{
					return keys[i][c];
				}
			}
			gpio_set_value(colPins[c], HIGH);
		}
	}
	return 'E';
}
