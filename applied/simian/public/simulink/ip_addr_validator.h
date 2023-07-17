#pragma once
#include <algorithm>
#include <cctype>
#include <locale>
#include <string>

bool isValidIPv4(std::string ip_addr);

// Trimming functions for whitespace

void ltrim(std::string& s);
void rtrim(std::string& s);
void trim(std::string& s);
