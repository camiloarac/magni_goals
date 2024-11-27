#include "magni_goals/read_csv.h"

std::string_view CsvRow::operator[](std::size_t index) const {
    return std::string_view(
        &line_[separator_pos_[index] + 1],
        separator_pos_[index + 1] - (separator_pos_[index] + 1));
}

std::size_t CsvRow::size() const { return separator_pos_.size() - 1; }

std::istream& CsvRow::readNextRow(std::istream& str) {
    std::getline(str, line_);
    while (line_.substr(0, 2) == "//") std::getline(str, line_);
    separator_pos_.clear();
    // To point to the first character of the line when using []
    separator_pos_.emplace_back(-1);
    std::string::size_type pos = 0;
    while ((pos = line_.find(',', pos)) != std::string::npos) {
        separator_pos_.emplace_back(pos);
        ++pos;
    }
    // This checks for a trailing comma with no data after it.
    pos = line_.size();
    separator_pos_.emplace_back(pos);
    return str;
}