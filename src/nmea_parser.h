#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include "gps_common/GPSFix.h"

using namespace std;

class NMEAParser
{

public:
  NMEAParser() {}
  ~NMEAParser() {}

  void addByte(const char& byte);

  bool isReadyToParse() const { return ready_to_parse_; }
  bool parse(const string& nmea_string);
  bool parse();

  gps_common::GPSFix getMessage() const { return message_; }
  std::string getMessageType() const { return message_type_; }
  void clear();

  static string calculateChecksum(const string& str);

private:
  static const char MessageHeadSymbol = '$';
  static const char MessageTailSymbol = '\n';

  typedef void (NMEAParser::*ExtractionFunctionPtr)(const vector<string>&);
  map<string, ExtractionFunctionPtr> DataExtractors = {
    { "RMC", &NMEAParser::extractFromRMC },
    { "GGA", &NMEAParser::extractFromGGA },
    { "GSA", &NMEAParser::extractFromGSA },
  };

  std::vector<uint8_t> input_;
  bool ready_to_parse_ = false;

  std::string message_type_ = "";
  gps_common::GPSFix message_;

  void extractFromRMC(const vector<string>& tokens);
  void extractFromGGA(const vector<string>& tokens);
  void extractFromGSA(const vector<string>& tokens);

  void extractTime(const string& data);
  void extractLatitude(const string& data, const string& direction);
  void extractLongitude(const string& data, const string& direction);
  void extractVelocity(const string& data);
  void extractCourse(const string& data);

  void clearString(string& str);
  bool retrieveMessageType(const string& str);
  bool retrieveSentenceAndChecksum(const string& str, string& sentence, string& checksum);

  static vector<string> split(const string& str, const string& delimeter);
};

#endif // NMEAPARSER_H
