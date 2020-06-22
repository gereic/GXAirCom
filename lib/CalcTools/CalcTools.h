#include <Arduino.h>

#define pi 3.14159265358979323846
#define d2r 0.0174532925199433

/*
byte getVal(char c);
void Hex2Bin(String str, byte *buffer);
String Bin2Hex(byte *buffer);
IPAddress getClientNumber ( int clientNo );
int client_status();
*/
double deg2rad(double);
double rad2deg(double);
/*
double haversine_km(double lat1, double long1, double lat2, double long2);
*/
double distance(double lat1, double lon1, double lat2, double lon2, char unit);
/*
double deg2rad(double deg);
double rad2deg(double rad);
*/
double dtorA(double fdegrees);
double rtodA(double fradians);
int CalcBearingA(double lat1, double lon1, double lat2, double lon2);
/*
bool isNumeric( String inS);
int isPilotNESW(char NS, char EW );
String addDot ( String numbersIn, int dotPos);
String reformatLatLong ( String numbersIn, int digits);
*/









