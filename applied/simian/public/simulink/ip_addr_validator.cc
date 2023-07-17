
#include "ip_addr_validator.h"

bool isValidIPv4(std::string ip_addr) {
  // IPV4 format is x.x.x.x where x is an integer must be less than 256.
  if (count(ip_addr.begin(), ip_addr.end(), '.') != 3) return false;

  int a, b, c, d;
  int scan_val = sscanf(ip_addr.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d);
  return scan_val == 4 && a < 256 && b < 256 && c < 256 && d < 256;
}

void ltrim(std::string& s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
void rtrim(std::string& s) {
  s.erase(
      std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
      s.end());
}

// trim from both ends (in place)
void trim(std::string& s) {
  ltrim(s);
  rtrim(s);
}
