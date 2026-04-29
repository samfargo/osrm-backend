#ifndef EXTRACTION_HELPER_FUNCTIONS_HPP
#define EXTRACTION_HELPER_FUNCTIONS_HPP

#include <boost/algorithm/string/replace.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <string>

#include "guidance/parsing_toolkit.hpp"

namespace osrm::extractor
{

namespace detail
{

inline bool isDigits(const std::string &s)
{
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

inline bool parseUnsigned(const std::string &s, unsigned &out)
{
    if (!isDigits(s))
        return false;
    const auto value = std::strtoull(s.c_str(), nullptr, 10);
    if (value > std::numeric_limits<unsigned>::max())
        return false;
    out = static_cast<unsigned>(value);
    return true;
}

inline std::string upper(const std::string &input)
{
    std::string out = input;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return out;
}

inline bool parseOsmTime(const std::string &s, unsigned &out)
{
    if (s.empty())
        return false;
    const auto first_colon = s.find(':');
    if (first_colon == std::string::npos)
    {
        unsigned minutes = 0;
        if (!parseUnsigned(s, minutes))
            return false;
        out = minutes * 60;
        return true;
    }

    const auto second_colon = s.find(':', first_colon + 1);
    if (second_colon == std::string::npos)
    {
        unsigned hours = 0;
        unsigned minutes = 0;
        if (!parseUnsigned(s.substr(0, first_colon), hours) ||
            !parseUnsigned(s.substr(first_colon + 1), minutes))
            return false;
        out = hours * 3600 + minutes * 60;
        return true;
    }

    if (s.find(':', second_colon + 1) != std::string::npos)
        return false;

    unsigned hours = 0;
    unsigned minutes = 0;
    unsigned seconds = 0;
    if (!parseUnsigned(s.substr(0, first_colon), hours) ||
        !parseUnsigned(s.substr(first_colon + 1, second_colon - first_colon - 1), minutes) ||
        !parseUnsigned(s.substr(second_colon + 1), seconds))
        return false;
    out = hours * 3600 + minutes * 60 + seconds;
    return true;
}

inline bool parseFixedTwo(const std::string &s, std::size_t pos, unsigned &out)
{
    if (pos + 2 > s.size())
        return false;
    const std::string chunk = s.substr(pos, 2);
    return parseUnsigned(chunk, out);
}

inline bool parseIsoAlternativeTime(const std::string &s, unsigned &out)
{
    if (s.size() != 7 || s[0] != 'T')
        return false;
    unsigned hh = 0, mm = 0, ss = 0;
    if (!parseFixedTwo(s, 1, hh) || !parseFixedTwo(s, 3, mm) || !parseFixedTwo(s, 5, ss))
        return false;
    if (hh >= 24 || mm >= 60 || ss >= 60)
        return false;
    out = hh * 3600 + mm * 60 + ss;
    return true;
}

inline bool parseIsoExtendedTime(const std::string &s, unsigned &out)
{
    if (s.size() != 9 || s[0] != 'T' || s[3] != ':' || s[6] != ':')
        return false;
    unsigned hh = 0, mm = 0, ss = 0;
    if (!parseFixedTwo(s, 1, hh) || !parseFixedTwo(s, 4, mm) || !parseFixedTwo(s, 7, ss))
        return false;
    if (hh >= 24 || mm >= 60 || ss >= 60)
        return false;
    out = hh * 3600 + mm * 60 + ss;
    return true;
}

inline bool parseNumberWithSuffix(const std::string &s,
                                  std::size_t &pos,
                                  char suffix,
                                  unsigned &out)
{
    std::size_t i = pos;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])))
        ++i;
    if (i == pos || i >= s.size() || s[i] != suffix)
        return false;
    if (!parseUnsigned(s.substr(pos, i - pos), out))
        return false;
    pos = i + 1;
    return true;
}

inline bool parseIsoStandardTime(const std::string &s, unsigned &out)
{
    if (s.empty() || s[0] != 'T')
        return false;
    std::size_t pos = 1;
    unsigned hours = 0, minutes = 0, seconds = 0;
    bool any = false;
    unsigned value = 0;

    if (parseNumberWithSuffix(s, pos, 'H', value))
    {
        hours = value;
        any = true;
    }
    if (parseNumberWithSuffix(s, pos, 'M', value))
    {
        minutes = value;
        any = true;
    }
    if (parseNumberWithSuffix(s, pos, 'S', value))
    {
        seconds = value;
        any = true;
    }
    if (!any || pos != s.size())
        return false;
    out = hours * 3600 + minutes * 60 + seconds;
    return true;
}

inline bool parseIsoDateAndOptionalTime(const std::string &s, unsigned &out)
{
    std::size_t pos = 0;
    unsigned date_seconds = 0;
    bool has_date = false;
    unsigned days = 0;
    if (parseNumberWithSuffix(s, pos, 'D', days))
    {
        has_date = true;
        date_seconds = days * 86400;
    }

    unsigned time_seconds = 0;
    bool has_time = false;
    if (pos < s.size() && s[pos] == 'T')
    {
        has_time = parseIsoStandardTime(s.substr(pos), time_seconds);
        if (!has_time)
            return false;
        pos = s.size();
    }

    if (pos != s.size() || (!has_date && !has_time))
        return false;

    out = date_seconds + time_seconds;
    return true;
}

inline bool parseIsoPeriod(const std::string &s, unsigned &out)
{
    if (s.size() < 2)
        return false;
    const std::string normalized = upper(s);
    if (normalized[0] != 'P')
        return false;
    const std::string tail = normalized.substr(1);
    if (tail.empty())
        return false;

    unsigned weeks = 0;
    std::size_t week_pos = 0;
    if (parseNumberWithSuffix(tail, week_pos, 'W', weeks) && week_pos == tail.size())
    {
        out = weeks * 604800;
        return true;
    }

    if (parseIsoAlternativeTime(tail, out) || parseIsoExtendedTime(tail, out) ||
        parseIsoDateAndOptionalTime(tail, out))
        return true;

    return false;
}

inline bool parseDurationImpl(const std::string &s, unsigned &out)
{
    return parseOsmTime(s, out) || parseIsoPeriod(s, out);
}

} // namespace detail

inline bool durationIsValid(const std::string &s)
{
    unsigned duration = 0;
    return detail::parseDurationImpl(s, duration);
}

inline unsigned parseDuration(const std::string &s)
{
    unsigned duration = 0;
    return detail::parseDurationImpl(s, duration) ? duration : std::numeric_limits<unsigned>::max();
}

inline std::string
trimLaneString(std::string lane_string, std::int32_t count_left, std::int32_t count_right)
{
    return guidance::trimLaneString(std::move(lane_string), count_left, count_right);
}

inline std::string applyAccessTokens(const std::string &lane_string,
                                     const std::string &access_tokens)
{
    return guidance::applyAccessTokens(lane_string, access_tokens);
}

// Takes a string representing a list separated by delim and canonicalizes containing spaces.
// Example: "aaa;bbb; ccc;  d;dd" => "aaa; bbb; ccc; d; dd"
inline std::string canonicalizeStringList(std::string strlist, const std::string &delim)
{
    // expand space after delimiter: ";X" => "; X"
    boost::replace_all(strlist, delim, delim + " ");

    // collapse spaces; this is needed in case we expand "; X" => ";  X" above
    // but also makes sense to do irregardless of the fact - canonicalizing strings.
    const auto spaces = [](unsigned char lhs, unsigned char rhs)
    { return ::isspace(lhs) && ::isspace(rhs); };
    auto it = std::unique(begin(strlist), end(strlist), spaces);
    strlist.erase(it, end(strlist));

    return strlist;
}

} // namespace osrm::extractor

#endif // EXTRACTION_HELPER_FUNCTIONS_HPP
