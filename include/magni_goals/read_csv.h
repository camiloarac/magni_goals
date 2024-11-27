#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

class CsvRow {
   public:
    std::string_view operator[](std::size_t index) const;
    std::size_t size() const;
    std::istream& readNextRow(std::istream& str);

   private:
    std::string line_;
    std::vector<int> separator_pos_;
};