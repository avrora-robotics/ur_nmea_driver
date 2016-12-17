#include "nmea_parser.h"

void NMEAParser::addByte(const char &byte)
{
  if (input_.size() == 0 && byte == NMEAParser::MessageHeadSymbol) {
    input_.push_back(byte);
    return;
  }

  if (input_.size() > 0) {
    input_.push_back(byte);

    if (byte == NMEAParser::MessageTailSymbol) {
      ready_to_parse_ = true;
    }
  }
}

bool NMEAParser::parse()
{
  std::string nmea_string(input_.begin(), input_.end());
  clearString(nmea_string);
  bool operation_result = parse(nmea_string);

  input_.clear();
  ready_to_parse_ = false;

  return operation_result;
}

bool NMEAParser::parse(const string& nmea_string)
{
  string sentence;
  string original_checksum;
  if (!NMEAParser::retrieveSentenceAndChecksum(nmea_string, sentence, original_checksum)) {
    ROS_WARN_STREAM("Cannot parse string: " << nmea_string);
    return false;
  }

  if (original_checksum != calculateChecksum(sentence)) {
    ROS_WARN_STREAM("Checksum does not match: " << nmea_string);
    return false;
  }

  vector<string> tokens = NMEAParser::split(sentence, ",");
  if (!retrieveMessageType(tokens.at(0))) {
    ROS_WARN_STREAM("Unknown NMEA message: " << nmea_string);
    return false;
  }

  tokens.erase(tokens.begin());
  try {
    (this->*(DataExtractors.at(message_type_)))(tokens);
  } catch (std::out_of_range) {
    // ROS_INFO_STREAM("Unknown NMEA protocol: " << message_type_);
    return false;
  }

  return true;
}

void NMEAParser::clear()
{
  message_ = gps_common::GPSFix();
}

void NMEAParser::extractFromRMC(const vector<string>& tokens)
{
  extractTime(tokens[0]);
  extractLatitude(tokens[2], tokens[3]);
  extractLongitude(tokens[4], tokens[5]);
  extractVelocity(tokens[6]);
  extractCourse(tokens[7]);
}

void NMEAParser::extractFromGGA(const vector<string>& tokens)
{
  extractTime(tokens[0]);
  extractLatitude(tokens[1], tokens[2]);
  extractLongitude(tokens[3], tokens[4]);

  stringstream(tokens[8]) >> message_.altitude;
}

void NMEAParser::extractFromGSA(const vector<string>& tokens)
{
  stringstream(tokens[14]) >> message_.pdop;
  stringstream(tokens[15]) >> message_.hdop;
  stringstream(tokens[16]) >> message_.vdop;
}

void NMEAParser::extractTime(const string& str)
{
  stringstream(str) >> message_.time;
}

void NMEAParser::extractLatitude(const string& data, const string& direction)
{
  message_.latitude = 0;

  if (data.size() > 4 && direction.size() > 0) {
    double degrees = 0;
    stringstream(data.substr(0, 2)) >> degrees;

    double minutes = 0;
    stringstream(data.substr(2, data.length() - 2)) >> minutes;

    message_.latitude = degrees + minutes / 60.0 * (direction[0] == 'N' ? 1 : -1);
  }
}

void NMEAParser::extractLongitude(const string& data, const string& direction)
{
  message_.longitude = 0;

  if (data.size() > 5 && direction.size() > 0) {
    double degrees = 0;
    stringstream(data.substr(0, 3)) >> degrees;

    double minutes = 0;
    stringstream(data.substr(3, data.length() - 3)) >> minutes;

    message_.longitude = degrees + minutes / 60.0 * (direction[0] == 'E' ? 1 : -1);
  }
}

void NMEAParser::extractVelocity(const string& data)
{
  static constexpr double KnotsToMetersPerSecond = 0.514444;

  stringstream(data) >> message_.speed;
  message_.speed *= KnotsToMetersPerSecond;
}

void NMEAParser::extractCourse(const string& data)
{
  stringstream(data) >> message_.track;
}

void NMEAParser::clearString(string &str)
{
  string::size_type nmea_start = str.find(NMEAParser::MessageHeadSymbol);
  str.erase(str.begin(), str.begin() + nmea_start + 1);
  str.erase(str.find(NMEAParser::MessageTailSymbol));
  boost::trim(str);
}

bool NMEAParser::retrieveMessageType(const string& str)
{
  const int TalkerIdentifierLength = 2;
  if (str.size() <= TalkerIdentifierLength) {
    return false;
  }

  string talker = str.substr(0, TalkerIdentifierLength);
  if (talker == "GP" || talker == "GL" || talker == "GN") {
    message_type_ = str.substr(TalkerIdentifierLength, str.size() - TalkerIdentifierLength);
  } else {
    message_type_ = str;
  }

  return true;
}

bool NMEAParser::retrieveSentenceAndChecksum(const string& str, string& sentence, string& checksum)
{
  const size_t PartsCount = 2;

  vector<string> tokens = NMEAParser::split(str, "*");
  if (tokens.size() != PartsCount) {
    return false;
  }

  sentence = tokens.at(0);
  checksum = tokens.at(1);

  return true;
}

string NMEAParser::calculateChecksum(const string& str)
{
  int checksum = 0;
  for (char symbol : str) {
    checksum ^= symbol;
  }

  stringstream stream;
  stream << std::hex << checksum;

  string result = stream.str();
  for (size_t i = 0; i < 2 - result.length(); i++) {
    result.insert(0, "0");
  }

  transform(result.begin(), result.end(),result.begin(), (int (*)(int))toupper);

  return result;
}

vector<string> NMEAParser::split(const string& str, const string& delimeter)
{
  vector<string> tokens;
  string input_copy = str;

  size_t current_position = 0;
  while ((current_position = input_copy.find(delimeter)) != std::string::npos) {
    tokens.push_back(input_copy.substr(0, current_position));
    input_copy.erase(0, current_position + delimeter.length());
  }

  if (input_copy.size() > 0) {
    tokens.push_back(input_copy);
  }

  return tokens;
}
