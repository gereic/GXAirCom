#include "MicroNMEA.h"
#include <iostream>
#include <iomanip>
using namespace std;

const char* separator = "--------------";


void unknownSentenceHandler(MicroNMEA &nmea)
{
  cout << "Unknown sentence: \"" << nmea.getSentence() << '"' << endl
       << separator << endl;
}

void badChecksumHandler(MicroNMEA &nmea)
{
  // Don't print messages for empty lines or comments
  const char *s = nmea.getSentence(); 
  if (s && strlen(s) && s[0] == '$' ) {
    char checksum[3];
    MicroNMEA::generateChecksum(s, checksum);
    checksum[2] = '\0';
    cout << "Bad checksum for \"" << nmea.getSentence() << '"' << endl
	 << "Checksum should be " << checksum << endl
	 << separator << endl;
  }
}

ostream& formatDateTime(ostream& s, MicroNMEA& nmea)
{
  s << setfill('0')
    << setw(4) << nmea.getYear() << '-'
    << setw(2) << int(nmea.getMonth()) << '-'
    << setw(2) << int(nmea.getDay()) << 'T'
    << setw(2) << int(nmea.getHour()) << ':'
    << setw(2) << int(nmea.getMinute()) << ':'
    << setw(2) << int(nmea.getSecond()) << '.'
    << setw(2) << int(nmea.getHundredths());
  return s;
}

int main(void)
{
  char buffer[85];
  MicroNMEA nmea(buffer, sizeof(buffer));
  
  nmea.setBadChecksumHandler(badChecksumHandler);
  nmea.setUnknownSentenceHandler(unknownSentenceHandler);
  
  while (!cin.eof()) {
    char c;
    cin.get(c);
    cout.put(c);
    cout.flush();
    if (nmea.process(c)) {
      if (nmea.isValid()) {
	cout << "Sentence    " << nmea.getSentence() << endl
	     << "Valid        " << (nmea.isValid() ? "true" : "false") << endl
	     << "TalkerID     " << nmea.getTalkerID() << endl
	     << "MessageID    " << nmea.getMessageID() << endl;

	cout << "Date/time    ";
	formatDateTime(cout, nmea);
	cout << endl;
	
	cout << fixed << setprecision(6)
	     << "Latitude     " << nmea.getLatitude()
	     << " (" << (nmea.getLatitude() / 1e6) << " deg)" << endl
	     << "Longitude    " << nmea.getLongitude()
	     << " (" << (nmea.getLongitude() / 1e6) << " deg)" << endl;

	long altitude;
	if (nmea.getAltitude(altitude))
	  cout << fixed << setprecision(3)
	       << "Alt. valid   true" << endl
	       << "Altitude     " << altitude << " ("
	       << fixed << setprecision(3) << (altitude/1e3) << "m)" << endl;
	else
	  cout << "Alt. valid   false" << endl;


	cout << separator << endl;
      }
    }
  }

}
