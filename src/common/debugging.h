/*
   mkvmerge -- utility for splicing together matroska files
   from component media subtypes

   Distributed under the GPL
   see the file COPYING for details
   or visit http://www.gnu.org/copyleft/gpl.html

   definitions used in all programs, helper functions

   Written by Moritz Bunkus <moritz@bunkus.org>.
*/

#ifndef MTX_COMMON_DEBUGGING_H
#define MTX_COMMON_DEBUGGING_H

#include "common/common_pch.h"

bool debugging_requested(const char *option, std::string *arg = nullptr);
bool debugging_requested(const std::string &option, std::string *arg = nullptr);
void request_debugging(const std::string &options);
void init_debugging();

int parse_debug_interval_arg(const std::string &option, int default_value = 1000, int invalid_value = -1);

class debugging_option_c {
protected:
  boost::tribool m_requested;
  std::string m_option;

public:
  debugging_option_c(std::string const &option)
    : m_requested{boost::logic::indeterminate}
    , m_option{option}
  {
  }

  operator bool() {
    if (boost::logic::indeterminate(m_requested))
      m_requested = debugging_requested(m_option);

    return m_requested;
  }
};

#endif  // MTX_COMMON_DEBUGGING_H
