/******************************************************************************

GeoCache Hunt Project (GeoCache.cpp)
  
This is skeleton code is provided to the work to be done.  You are not 
required to follow this coding structure.  You are free to implement
your project however you wish.

Consider using sprintf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need to convert it to Decimal Degrees format (DDD.DDDD).

*******************************************************************************
Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must figure out how to parse the message to obtain the
parameters required for the GeoCache project.  Additional information on the
GPS device and messaging can be found in the documents supplied in your resource
coordinates in the following GPRMC sample message, after convert to Decimal
Degrees (DDD.DDDDDD) as latitude(23.118757) longitude(120.274060).  By the way,
this coordinate is GlobalTop Technology in Tiawan, who designed and manufactured
the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C\r\n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // coordinate status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm) range[0.90]
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm) range[0.180]
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
\r\n            // return and newline

*******************************************************************************

Configuration settings.

These defines make it easy for you to enable/disable certain
code during the development and debugging cycle of this project.

The results below are calculated from above GPS GPRMC message
and the GEOLAT0/GEOLON0 tree as target.  Your results should be
nearly identical, if not exactly the same.

Results of converting GPS LAT or LON string to a float:
LAT_2307.1256 = 2307.125488
LON_12016.4438 = 12016.443359

Results of executing the following functions:
degMin2DecDeg() LAT_2307.1256_N = 23.118757 decimal degrees
degMin2DecDeg() LON_12016.4438_E = 120.274055 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999652 degrees

Results for adjusting for relative bearing towards tree = 217.519650 degrees

******************************************************************************/

#include <SD.h>
#include "wiring_private.h"
#include <Adafruit_seesaw.h>
#include <Adafruit_SH110X.h>

// compile flags
#define GPS_ON 1		// GPS messages classroom testing=0, live outside=1
#define LOG_ON 0

// Feather PINS for peripherals
#define M0TX_PIN	10	// MO TX -> GPS RX
#define M0RX_PIN	11	// MO RX <- GPS TXF
#define SDC_CS	4		  // Secure Digital Card SPI chip select
#define BAT_IN	A7		// Battery analog pin

// joy BITS for buttons
#define BUT_RT	(1<<6)
#define BUT_DN	(1<<7)
#define BUT_LF	(1<<9)
#define BUT_UP	(1<<10)
#define BUT_SL	(1<<14)
#define BUT_MSK (BUT_RT|BUT_DN|BUT_LF|BUT_UP|BUT_SL)

#define GPS_BUFSIZ	96	// max size of GPS char buffer

// OLED PINS (not used)
// #define BUT_A 	9
// #define BUT_B 	6
// #define BUT_C	5

// GPS control messages
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

/****
Sample coordinates that can be used for testing.
****/

// FS3B-116 large tree outside front door
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

// grass between taco bell and office
#define GEOLAT1 28.596556
#define GEOLON1 -81.306430

// FS2 rear parking retention pond
#define GEOLAT2 28.595888
#define GEOLON2 -81.301271

// Front FS4 436 entrance
#define GEOLAT3 28.591209
#define GEOLON3 -81.306019

// waypoint structure
typedef struct
{
	float latitude;
	float longitude;
} WAYPOINT;

/**************************/
/**** GLOBAL VARIABLES ****/
/**************************/

bool recording = false;	// SDC is recording
bool acquired = false;	// GPS acquired position

uint8_t target = 0;		// GeoCache target number
float flat = 0.0;		// current GPS latitude position
float flon = 0.0;		// current GPS longitude position
float fcog = 0.0;		// current course over ground
float fbrg = 0.0;		// true north target bearing
float frel = 0.0;		// relative bearing to target
float fdis = 0.0;		// current distance to target

/************************/
/**** GLOBAL OBJECTS ****/
/************************/
//Flag 0 (Final)
#define LAT0 28.59574
#define LON0 -81.3052

//Flag 1 (Final)
#define LAT1 28.59641
#define LON1 -81.30081

//Flag 2 (Final)
#define LAT2 28.59082
#define LON2 -81.30391

//Flag 3 (Final)
#define LAT3 28.59326
#define LON3 -81.30323

WAYPOINT waypoint[] = 
{
	LAT0, LON0,
	LAT1, LON1,
	LAT2, LON2,
	LAT3, LON3,
};

File logFile;
Adafruit_seesaw joy;
Adafruit_SH1107 oled = Adafruit_SH1107(64, 128, &Wire);
Uart gps (&sercom1, M0RX_PIN, M0TX_PIN, SERCOM_RX_PAD_0, UART_TX_PAD_2);

// gps serial interrupt handler
void SERCOM1_Handler(void)
{
  gps.IrqHandler();
}

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *ccor, char *cind)

Input:
	ccor = char string pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate
	cind = char string pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator

Return:
	Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *ccor, char *cind)
{
	float degrees = 0.0;
  float degMin = atof(ccor);

	/*
		convert degrees minutes to decimal degrees
	*/
  uint8_t deg = degMin / 100;
  float min = (degMin - (deg * 100.0)) / 60.00;
  degrees = deg + min;
  if (*cind == 'S' || *cind == 'W')
    degrees *= -1;

#if LOG_ON
	Serial.print("degMin2DecDeg() returned: ");
	Serial.println(degrees, 6);
#endif

  return(degrees);
}

/**************************************************
Calculate Great Circle Distance between two coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
	flat1, flon1 = GPS latitude and longitude coordinate in decimal degrees
	flat2, flon2 = Target latitude and longitude coordinate in decimal degrees

Return:
	distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;


	/*
		calculated distance to target
	*/
  float a = pow(sin(((flat2 - flat1) * DEG_TO_RAD) / 2.0), 2.0) + cos(flat1 * DEG_TO_RAD) * cos(flat2 * DEG_TO_RAD) * pow(sin(((flon2 - flon1) * DEG_TO_RAD) / 2.0), 2.0);
  distance = (3959.0 * 5280.0) * (2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
	
#if LOG_ON
	Serial.print("calcDistance() returned: ");
	Serial.println(distance, 6);
#endif

	return(distance);
}

/******************************************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
	flat1, flon1 = gps latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
	angle in decimal degrees from magnetic north

NOTE: atan2() returns range of -pi/2 to +pi/2)

******************************************************************************/
/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
	flat1, flon1 = gps latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
	angle in decimal degrees from magnetic north
	
NOTE: atan2() returns range of -pi/2 to +pi/2)

**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;
  float deltaLon = (flon2 - flon1) * DEG_TO_RAD;
	flat1 *= DEG_TO_RAD;
  flat2 *= DEG_TO_RAD;
  flon1 *= DEG_TO_RAD;
  flon2 *= DEG_TO_RAD;

	/*
		calculate bearing to target
	*/
  float y = sin(deltaLon) * cos(flat2);
  float x = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(deltaLon);
  float theta = atan2(y, x);
  bearing = fmod((theta * RAD_TO_DEG) + 360.0, 360.0);

#if LOG_ON
	Serial.print("calcBearing() returned: ");
	Serial.println(bearing, 6);
#endif

	return(bearing);
}

/**************************************************
Calculate The relavtive bearing from the current true north target bearing

float calcRelativeBearing(float curbearing)

Input:
  curBearing = true north target bearing

Return:
	relative bearing within range of 0 - 360
**************************************************/
float calcRelativeBearing(float curBearing) {	
	/*
		calculate relative bearing
	*/
  float relBearing = fmod(((curBearing - fcog) + 360.0), 360.0);

#if LOG_ON
	Serial.print("calcRelativeBearing() returned: ");
	Serial.println(relBearing, 6);
#endif

	return(relBearing);
}

#if GPS_ON
/*
Get valid GPS message.

char* getGpsMessage(void)

Side affects:
Message is placed in local static char buffer.

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/

char* getGpsMessage(void)
{
	bool rv = false;
	static uint8_t x = 0;
	static char cstr[GPS_BUFSIZ];

	// get nmea string
	while (gps.peek() != -1)
	{
		// reset or bound cstr
		if (x == 0) memset(cstr, 0, sizeof(cstr));
		else if (x >= (GPS_BUFSIZ - 1)) x = 0;

		// read next char
		cstr[x] = gps.read();

		// looking for "$GPRMC", toss out undesired messages
		if ((x >= 3) && (cstr[0] != '$') &&  (cstr[3] != 'R'))
		{
			x = 0;
			break;
		}

		// if end of message received (sequence is \r\n)
		if (cstr[x] == '\n')
		{
			// nul terminate char buffer (before \r\n)
			cstr[x - 1] = 0;

			// if checksum not found
			if (cstr[x - 4] != '*')
			{
				x = 0;
				break;
			}

			// convert hex checksum to binary
			uint8_t isum = strtol(&cstr[x - 3], NULL, 16);

			// reverse checksum
			for (uint8_t y = 1; y < (x - 4); y++) isum ^= cstr[y];

			// if invalid checksum
			if (isum != 0)
			{
				x = 0;
				break;
			}

			// else valid message
			rv = true;
			x = 0;
			break;
		}

		// increment buffer position
		else x++;

		// software serial must breath, else miss incoming characters
		delay(1);
	}

	if (rv) return(cstr);
	else return(nullptr);
}

#else
/*
Get simulated GPS message provided once a second.

This is the same message and coordinates as described at the top of this
file.

NOTE: DO NOT CHANGE THIS CODE !!!

char* getGpsMessage(void)

Side affects:
Message is place in local static char buffer

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/
char* getGpsMessage(void)
{
	static char cstr[GPS_BUFSIZ];
	static uint32_t timestamp = 0;
	uint32_t timenow = millis();

	// provide message every second
	if (timestamp >= timenow) return(nullptr);

	String sstr = "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C";

	memcpy(cstr, sstr.c_str(), sstr.length());

	timestamp = timenow + 1000;

	return(cstr);
}

#endif

float getBatteryVoltage(void)
{
	float vbat = analogRead(BAT_IN);
	vbat *= 2;    // normalize - input is divided by 2 using restor divider.
	vbat *= 3.3;  // multiply by analog input reference voltage of 3.3v.
	vbat /= 1024; // convert to actual battery voltage (10 bit analog input)
	return(vbat);
}

void setup(void)
{
	// delay till terminal opened
	Serial.begin(115200);

  // wait upto 5 seconds to open serial terminal
  if (LOG_ON)
    while(!Serial && (millis() < 5000));

	// initialize status LED=OFF
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// initialize oled dispaly
	while (oled.begin(0x3C, true) == false)
	{
		Serial.println("oled.begin() failed");
		delay(1000);
	}

	/********************************
	oled is 128w x 64h
	Chars initialized to 6w x 8h
	Max 21 chars per line
	Max 8 lines per display
	********************************/
	oled.clearDisplay();
  oled.setTextSize(1);
	oled.setRotation(1);
	oled.setTextColor(SH110X_WHITE);
	oled.setCursor(0, 0);
	oled.println("GeoCache Hello World!");
	oled.display();

	// initialize joy board
	while (joy.begin() == false)
	{
		Serial.println("joy.begin() failed");
		delay(1000);
	}

	// set joy pin modes/interrupts
	joy.pinModeBulk(BUT_MSK, INPUT_PULLUP);
	joy.setGPIOInterrupts(BUT_MSK, true);
  pinMode(5, INPUT_PULLUP);

	// initialize Secure Digital Card and open "MyMap.txt" file for writing
  if (GPS_ON) {
    while (!SD.begin(SDC_CS));
    logFile = SD.open("MyMap.txt", FILE_WRITE);
  }

#if GPS_ON
	// initilaze gps serial baud rate
	gps.begin(9600);

	// map rx/tx pins to sercom
	pinPeripheral(M0RX_PIN, PIO_SERCOM);
  pinPeripheral(M0TX_PIN, PIO_SERCOM);

	// initialize gps message type/rate
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif

	Serial.println("setup() complete");
}

void loop(void)
{
  static uint32_t curTime = 0;
  uint32_t timenow = millis();
  static uint8_t halfSelectedFlag = 0;

	// get GPS message
	char* cstr = getGpsMessage();

	// if valid message
	if (cstr)
	{
		// print the GPRMC message
		Serial.println(cstr);	

		// Check button for incrementing target index 0..3
    uint16_t buttonStates = joy.digitalReadBulk(BUT_MSK);
    if (!(buttonStates & BUT_UP))
      halfSelectedFlag = 0;
    if (!(buttonStates & BUT_RT))
      halfSelectedFlag = 1;
    if (!(buttonStates & BUT_DN))
      halfSelectedFlag = 2;
    if (!(buttonStates & BUT_LF))
      halfSelectedFlag = 3;
    if (!(buttonStates & BUT_SL))
      target = halfSelectedFlag;

		// Parse 5 parameters latitude, longitude, and hemisphere indicators, and course over ground from GPS message
    strtok(cstr, ","); strtok(NULL, ","); 
    char *validData = strtok(NULL, ",");
    if (*validData == 'A')
      acquired = true;
    else
      acquired = false;

		// Call degMin2DecDeg() convert latitude deg/min to dec/deg
    flat = degMin2DecDeg(strtok(NULL, ","), strtok(NULL, ","));

		// Call degMin2DecDeg() convert longitude deg/min to dec/deg
    flon = degMin2DecDeg(strtok(NULL, ","), strtok(NULL, ","));

    // Filter out course over ground and print, along with lat and lon (in dec/deg)
    strtok(NULL, ",");
    fcog = atof(strtok(NULL, ","));
    Serial.println("Lat: " + String(flat, 6) + " | Lon: " + String(flon, 6) + " | Course Over Ground: " + String(fcog, 6) + " | Flag: " + String(target));

		// Call calcDistance() calculate distance to target
		fdis = calcDistance(flat, flon, waypoint[target].latitude, waypoint[target].longitude);

		// Call calcBearing() calculate bearing to target to call in relative bearing
    fbrg = calcBearing(flat, flon, waypoint[target].latitude, waypoint[target].longitude);

		// Calculate relative bearing within range >= 0 and < 360
    frel = calcRelativeBearing(fbrg);
		
  #if LOG_ON 
		Serial.print("Relative Bearing: ");
    Serial.println(frel);
  #endif
		
		// write required data to SecureDigital then execute flush()
    if (acquired && GPS_ON && !recording) {
      recording = true;
      logFile.println(String(flon, 6) + "," + String(flat, 6) + "," + String((int)frel) + "." + String((int)fdis));
      logFile.flush();
      recording = false;
    }

		/* Display
        Target Number
        Relative Bearing to Target
        Distance to Target
        Battery Voltage
    */
    oled.clearDisplay();
    oled.setCursor(0, 0);   
    oled.println("Flag: " + String(target) + (target == halfSelectedFlag ? "" : "->" + String(halfSelectedFlag)));
    oled.println("Rel. Bearing: " + (acquired ? String(frel) : "N/A"));
    oled.println("Distance: " + (acquired ? String(fdis) : "N/A"));
    oled.println("Bat. V: " + String(getBatteryVoltage()));
    oled.display();
	}

  // toggle LED_BUILTIN once a second.
  if (timenow - curTime >= 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    curTime = millis();
  }
}