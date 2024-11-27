#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

class CsvRow
{
public:
  std::string_view operator[](std::size_t index) const;
  std::size_t size() const;
  std::istream& readNextRow(std::istream& str);

private:
  std::string m_line;
  std::vector<int> m_data;
};

// std::istream& operator>>(std::istream& str, CsvRow& data)
// {
//   data.readNextRow(str);
//   return str;
// }