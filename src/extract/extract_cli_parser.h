/*
   mkvextract -- extract tracks from Matroska files into other files

   Distributed under the GPL
   see the file COPYING for details
   or visit http://www.gnu.org/copyleft/gpl.html

   Written by Moritz Bunkus <moritz@bunkus.org>.
*/

#ifndef __EXTRACT_EXTRACT_CLI_PARSER_H
#define __EXTRACT_EXTRACT_CLI_PARSER_H

#include "common/os.h"

#include <string>
#include <vector>

#include "common/cli_parser.h"
#include "extract/mkvextract.h"
#include "extract/options.h"

class extract_cli_parser_c: public cli_parser_c {
protected:
  options_c m_options;
  int m_num_unknown_args;

  std::string m_charset;
  bool m_embed_in_ogg;
  bool m_extract_cuesheet;
  int m_extract_blockadd_level;
  track_spec_t::target_mode_e m_target_mode;

public:
  extract_cli_parser_c(const std::vector<std::string> &args);

  options_c run();

protected:
  void init_parser();

  void assert_mode(options_c::extraction_mode_e mode);

  void set_parse_fully();
  void set_charset();
  void set_no_ogg();
  void set_cuesheet();
  void set_blockadd();
  void set_raw();
  void set_fullraw();
  void set_simple();
  void set_mode_or_extraction_spec();
  void set_extraction_mode();
  void add_extraction_spec();
};

#endif // __EXTRACT_EXTRACT_CLI_PARSER_H